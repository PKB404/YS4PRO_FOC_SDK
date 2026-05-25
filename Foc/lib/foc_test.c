#include "foc_test.h"
#include "board.h"
#include "bsp_encoder.h"
#include "stm32f407xx.h"
#include "stm32f4xx_it.h"
#include "tim.h"
#include "usart.h"
#include <math.h>
#include "usbd_cdc_if.h"
#include "foc_math.h"

#define _2PI            6.283185307f

typedef struct {
    float Kp;       // 比例系数
    float Ki;       // 积分系数（未乘 dt）
    float integral; // 积分累积
    float out_max;  // 输出限幅上限
    float out_min;  // 输出限幅下限
    float dt;       // 控制周期（秒）
} SpeedPI;

Bsp_encoder_t *g_enc;
FOC_PWM_t g_FOC_PWM;
SpeedPI g_speed_pi;
float g_target_speed_rad_s;
float g_openloop_vq = 0.5f; /* 开环Vq给定(V) */
float speed_fb;
float ele_angle;
float delta;
float vofa_floatdata[4] = {0.0f};
static bool g_calibrated = false; /* 是否已完成校准 */

static float rpm_to_elec_rad_s(float rpm, uint8_t pole_pairs) { return rpm * 0.104719755f * (float)pole_pairs; }

/* ============ FOC 数学变换 ============ */

Clarke_ab_t FOC_Clarke(PhaseCurrents_t *pParam) {
    Clarke_ab_t Output;

    Output.alpha = pParam->a;
    Output.beta = (pParam->a + 2.0f * pParam->b) * _1_DIV_SQRT_3;

    return Output;
}

Park_dq_t FOC_Park(Clarke_ab_t *pParam, float angle_el) {
    Park_dq_t Output;

    float sin = sinf(angle_el);
    float cos = cosf(angle_el);

    Output.d = pParam->alpha * cos + pParam->beta * sin;
    Output.q = pParam->beta * cos - pParam->alpha * sin;

    return Output;
}

PhaseCurrents_t FOC_InvClarke(Clarke_ab_t *pParam) {
    PhaseCurrents_t Output;

    Output.a = pParam->alpha;
    Output.b = -0.5f * pParam->alpha + SQRT3_DIV_2 * pParam->beta;
    Output.c = -0.5f * pParam->alpha - SQRT3_DIV_2 * pParam->beta;

    return Output;
}

Clarke_ab_t FOC_InvPark(Park_dq_t *pParam, float angle_el) {
    Clarke_ab_t Output;

    float sin = sinf(angle_el);
    float cos = cosf(angle_el);

    Output.alpha = pParam->d * cos - pParam->q * sin;
    Output.beta = pParam->d * sin + pParam->q * cos;

    return Output;
}

/* ============ PWM 底层 ============ */

void FOC_PWM_SetCompare(uint32_t ccr1, uint32_t ccr2, uint32_t ccr3) {
    htim1.Instance->CCR1 = ccr1;
    htim1.Instance->CCR2 = ccr2;
    htim1.Instance->CCR3 = ccr3;
}

/* ============ SVPWM ============ */

// uint8_t SectorJudgment(Clarke_ab_t *pAlphabeta) {
//     float A = pAlphabeta->beta;
//     float B = SQRT_3 * pAlphabeta->alpha - pAlphabeta->beta;
//     float C = -SQRT_3 * pAlphabeta->alpha - pAlphabeta->beta;

//     uint8_t weight = 0, sector = 0;

//     if (A > 0)
//         weight += 1;
//     if (B > 0)
//         weight += 2;
//     if (C > 0)
//         weight += 4;

//     switch (weight) {
//     case 3:
//         sector = 1;
//         break;
//     case 1:
//         sector = 2;
//         break;
//     case 5:
//         sector = 3;
//         break;
//     case 4:
//         sector = 4;
//         break;
//     case 6:
//         sector = 5;
//         break;
//     case 2:
//         sector = 6;
//         break;
//     default:
//         break;
//     }
//     return sector;
// }

/**
 * @brief 扇区判断（与下方的 X,Y,Z 计算完全匹配）
 * @param pAlphabeta 输入 alpha, beta 分量
 * @return uint8_t 扇区编号 1~6
 */
static uint8_t SectorJudgment(Clarke_ab_t *pAlphabeta)
{
    static const uint8_t sector_map[8] = {0, 1, 5, 0, 3, 2, 4, 0};
    float Ua = pAlphabeta->alpha;
    float Ub = pAlphabeta->beta;
    uint8_t weight = 0, sector = 0;

    // 计算三个辅助变量（与 VectorActionTime 中的 X,Y,Z 符号关系一致）
    float X = Ub;
    float Y = SQRT3_DIV_2 * Ua + 0.5f * Ub;
    float Z = -SQRT3_DIV_2 * Ua + 0.5f * Ub;

    // 根据符号位组合成 1~6 的扇区
    uint8_t a = X > 0 ? 1 : 0;
    uint8_t b = Y > 0 ? 1 : 0;
    uint8_t c = Z > 0 ? 1 : 0;
    weight = (a << 2) | (b << 1) | c;

    sector = sector_map[weight];
    return sector;
}

static void VectorActionTime(FOC_PWM_t *pFOC_PWM, uint8_t sector, Clarke_ab_t *pAlphabeta) {
    float T0 = 0.0f, T1 = 0.0f, T2 = 0.0f, sum;
    float vbus = pFOC_PWM->bus_Voltage;
    uint32_t Tpwm = pFOC_PWM->wave_period;

    PhaseCurrents_t Tabc_0 = {0};
    PhaseCurrents_t Tabc_1 = {0};

    float tmp = (float)Tpwm * SQRT_3 / vbus;

    float x = tmp * pAlphabeta->beta;
    float y = tmp * (SQRT3_DIV_2 * pAlphabeta->alpha + pAlphabeta->beta * 0.5f);
    float z = tmp * (-SQRT3_DIV_2 * pAlphabeta->alpha + pAlphabeta->beta * 0.5f);

    switch (sector) {
        case 1:
            T1 = -z;
            T2 = x;
            break;
        case 2:
            T1 = z;
            T2 = y;
            break;
        case 3:
            T1 = x;
            T2 = -y;
            break;
        case 4:
            T1 = -x;
            T2 = z;
            break;
        case 5:
            T1 = -y;
            T2 = -z;
            break;
        case 6:
            T1 = y;
            T2 = -x;
            break;
        default:
            T1 = 0;
            T2 = 0;
            break;
    }

    /* 限幅 */
    sum = T1 + T2;
    if (sum > Tpwm) {
        T1 = T1 / sum * Tpwm;
        T2 = T2 / sum * Tpwm;
    }
    T0 = Tpwm - T1 - T2;

    /* 7 段式 SVPWM */
    Tabc_0.a = T0 / 4.0f;
    Tabc_0.b = Tabc_0.a + T1 / 2.0f;
    Tabc_0.c = Tabc_0.b + T2 / 2.0f;

    switch (sector) {
    case 1:
        Tabc_1.a = Tabc_0.b;
        Tabc_1.b = Tabc_0.a;
        Tabc_1.c = Tabc_0.c;
        break;
    case 2:
        Tabc_1.a = Tabc_0.a;
        Tabc_1.b = Tabc_0.c;
        Tabc_1.c = Tabc_0.b;
        break;
    case 3:
        Tabc_1.a = Tabc_0.a;
        Tabc_1.b = Tabc_0.b;
        Tabc_1.c = Tabc_0.c;
        break;
    case 4:
        Tabc_1.a = Tabc_0.c;
        Tabc_1.b = Tabc_0.b;
        Tabc_1.c = Tabc_0.a;
        break;
    case 5:
        Tabc_1.a = Tabc_0.c;
        Tabc_1.b = Tabc_0.a;
        Tabc_1.c = Tabc_0.b;
        break;
    case 6:
        Tabc_1.a = Tabc_0.b;
        Tabc_1.b = Tabc_0.c;
        Tabc_1.c = Tabc_0.a;
        break;

    default:
        break;
    }

    float cmpA = (float)Tpwm - 2.0f * Tabc_1.a;
    float cmpB = (float)Tpwm - 2.0f * Tabc_1.b;
    float cmpC = (float)Tpwm - 2.0f * Tabc_1.c;

    /* 限幅（防止负值或超出周期） */
    if (cmpA < 0.0f) cmpA = 0.0f;
    if (cmpA > Tpwm) cmpA = Tpwm;
    if (cmpB < 0.0f) cmpB = 0.0f;
    if (cmpB > Tpwm) cmpB = Tpwm;
    if (cmpC < 0.0f) cmpC = 0.0f;
    if (cmpC > Tpwm) cmpC = Tpwm;

    pFOC_PWM->Uabc.a = cmpA / (float)Tpwm * vbus;
    pFOC_PWM->Uabc.b = cmpB / (float)Tpwm * vbus;
    pFOC_PWM->Uabc.c = cmpC / (float)Tpwm * vbus;

    FOC_PWM_SetCompare(cmpA, cmpB, cmpC);
}


// void FOC_Run_SVPWM(FOC_PWM_t *pFOC_PWM) {
//     uint8_t sector;
//     Clarke_ab_t input = {0};

//     input = FOC_InvPark(&pFOC_PWM->Uqd, pFOC_PWM->angle_el);
//     pFOC_PWM->Ualpha_beta = input;

//     sector = SectorJudgment(&input);
//     VectorActionTime(pFOC_PWM, sector, &input);
// }




/* ============ 零点校准 ============ */

/**
 * @brief 电角度零点校准
 * @note  向D轴（电角度0°）注电锁轴 → 多次采样编码器角度求平均
 *        → 把平均值存为编码器零点偏移
 *        校准完成后标记 g_calibrated = true
 */
void FOC_Align_Zero(void) {
    float mech_rad;
    float sum = 0.0f;
    float avg_mech_rad;
    uint32_t half_period = 2625; /* ARR=5250/2, 中心对齐模式 */
    uint32_t delta;
    float align_ratio = 0.20f; /* 12% 母线电压锁轴 */

    /* ---- 步骤1: 先高阻态 ---- */
    FOC_PWM_SetCompare(half_period, half_period, half_period);
    HAL_Delay(50);

    /* ---- 步骤2: D轴注电，电角度=0° → A高 BC低 ---- */
    delta = (uint32_t)((float)half_period * align_ratio);
    FOC_PWM_SetCompare(half_period + delta, half_period - delta / 2, half_period - delta / 2);
    HAL_Delay(1000); /* 等待转子稳定，大惯量电机可加长 */

    /* ---- 步骤3: 多次采样求平均 ---- */
    for (int i = 0; i < 16; i++) {
        Bsp_Encoder_Update(g_enc);
        mech_rad = Bsp_Encoder_Get_Mech_Angle(g_enc);
        sum += mech_rad;
        HAL_Delay(50);
    }
    avg_mech_rad = sum / 8.0f;

    /* ---- 步骤4: 存入编码器 zero_offset 字段 ---- */
    Bsp_Encoder_Set_Zero_Offset(g_enc, avg_mech_rad * 57.29578f);

    /* ---- 步骤5: 释放PWM ---- */
    FOC_PWM_SetCompare(half_period, half_period, half_period);

    /* ---- 步骤6: 标记校准完成 ---- */
    g_calibrated = true;
}

/* ============ 开环启动 ============ */

/**
 * @brief 启动开环SVPWM控制
 * @param vq 开环Q轴给定电压(V)
 * @note  必须先调过 FOC_Align_Zero() 完成校准
 */
void FOC_Start_OpenLoop(float vq) {
    if (!g_calibrated)
        return;

    g_FOC_PWM.bus_Voltage = 12.0f;
    g_FOC_PWM.wave_period = 5250; /* = ARR+1, 跟TIM1配置一致 */
    g_openloop_vq = vq;

    FOC_PWM_SetCompare(2625, 2625, 2625);
}

/* ============ PI 速度控制器(备用) ============ */

float Speed_PI_Update(SpeedPI *pPI, float target, float feedback) {
    float error = target - feedback;

    float output = pPI->Kp * error + pPI->Ki * pPI->integral;

    if (output > pPI->out_max)
        output = pPI->out_max;
    else if (output < pPI->out_min)
        output = pPI->out_min;

    if ((output >= pPI->out_max && error > 0) || (output <= pPI->out_min && error < 0)) {
        /* 抗积分饱和：不累加 */
    } else {
        pPI->integral += error * pPI->dt;
    }

    float integral_max = pPI->out_max / pPI->Ki;
    float integral_min = pPI->out_min / pPI->Ki;
    if (pPI->integral > integral_max)
        pPI->integral = integral_max;
    if (pPI->integral < integral_min)
        pPI->integral = integral_min;

    return output;
}

/* ============ 初始化 ============ */

void FOC_Init(void) {
    /* TIM1 输出50% 占空比高阻态 */
    htim1.Instance->CCR1 = 2625;
    htim1.Instance->CCR2 = 2625;
    htim1.Instance->CCR3 = 2625;

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    /* 速度PI参数（备用） */
    g_speed_pi.Kp = 0.05f;
    g_speed_pi.Ki = 0.5f;
    g_speed_pi.dt = 0.001f;
    g_speed_pi.integral = 0.0f;
    g_speed_pi.out_max = 6.5f;
    g_speed_pi.out_min = -6.5f;

    /* 创建编码器实例 */
    g_enc = Bsp_Encoder_Create(7, false, 0.001f);
    HAL_TIM_Base_Start_IT(&htim7);

    /* TIM7 中断在校准后才启动，见 FOC_Start_OpenLoop() */
}

float Limit_Angle(float ElAngle) {
    while (ElAngle > _2PI || ElAngle < 0) {
        if (ElAngle > _2PI) {
            ElAngle -= _2PI;
        } else if (ElAngle < 0) {
            ElAngle += _2PI;
        }
    }

    return ElAngle;
}



/* 最大通道数 */
#define VOFA_MAX_CHANNELS 8

/* JustFloat协议帧尾(NaN值: 0x00,0x00,0x80,0x7F) */
#define VOFA_JUSTFLOAT_TAIL 0x7F800000U /* NaN作为帧尾标记 */

static uint8_t g_tx_buf[VOFA_MAX_CHANNELS * sizeof(float) + sizeof(float)];

int Vofa_Send_JustFloat(const float *values, uint8_t count) {
    uint32_t tail_value;
    uint32_t offset = 0;

    if (!values || count > VOFA_MAX_CHANNELS) {
        return -1;
    }

    memcpy(g_tx_buf, values, count * sizeof(float));
    offset += count * sizeof(float);

    tail_value = VOFA_JUSTFLOAT_TAIL;
    memcpy(&g_tx_buf[offset], &tail_value, sizeof(float));
    offset += sizeof(float);

    cdc_vcp_data_tx(g_tx_buf, (uint32_t)offset);
    // HAL_UART_Transmit(&huart1, g_tx_buf, offset, 50);
    return 0;
}



/* ============ TIM7 中断回调 ============ */

static float openloop_speed_hz = 0.0f; // 当前开环电频率(Hz)
static float target_speed_hz = 5.0f;   // 目标开环电频率(Hz)，7极对数下 5Hz 对应约 42 RPM，适合慢速观察
static float speed_ramp_step = 0.005f; // 每次中断频率的增加量(斜坡加速)

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM7) {
        if (Bsp_Encoder_Update(g_enc) != 0)
            return;

        if (openloop_speed_hz < target_speed_hz) {
            openloop_speed_hz += speed_ramp_step;
            if (openloop_speed_hz > target_speed_hz)
                openloop_speed_hz = target_speed_hz;
        }

        // 2. 根据 1ms (0.001s) 的周期，计算这一步该递增的角度
        // 增量 = 2 * PI * 频率 * dt
        float angle_step = 6.283185307f * openloop_speed_hz * 0.001f;
        delta += angle_step;

        if (delta > 6.283185307f) {
            delta -= 6.283185307f;
        }
        // /* 电角度(弧度)，已经减掉zero_offset，包含极对数 */
        // ele_angle = Bsp_Encoder_Get_Elec_Angle(g_enc);
        // float mech_angle = Bsp_Encoder_Get_Mech_Angle(g_enc);
        // /* 开环：Vd=0, Vq=给定 */
        // g_FOC_PWM.angle_el = delta; /* 弧度 */
        // g_FOC_PWM.Uqd.d = 0.0f;
        // g_FOC_PWM.Uqd.q = 4.0f; /* V */
        // g_FOC_PWM.bus_Voltage = 12.0f;
        // g_FOC_PWM.wave_period = 5250;

        // FOC_Run_SVPWM(&g_FOC_PWM);
        struct foc_dq v_dq;
        struct foc_ab v_ab;
        struct foc_pwm_out pwm_out;

        v_dq.d = 0.0f;
        v_dq.q = 1.0f / 12.0f;          /* 标幺化：目标Vq / 母线电压 */
        foc_angle_t angle_deg = delta * 57.29578f;  /* 弧度 → 度 */

        foc_inv_park(&v_ab, &v_dq, angle_deg);
        foc_svpwm(&pwm_out, &v_ab);

        uint32_t ccr_a = (uint32_t)(pwm_out.duty_a * 5250.0f);
        uint32_t ccr_b = (uint32_t)(pwm_out.duty_b * 5250.0f);
        uint32_t ccr_c = (uint32_t)(pwm_out.duty_c * 5250.0f);

        FOC_PWM_SetCompare(ccr_a, ccr_b, ccr_c);


        /* 调试输出(可选) */
        vofa_floatdata[0] = pwm_out.duty_a * 12.0f;
        vofa_floatdata[1] = pwm_out.duty_b * 12.0f;
        vofa_floatdata[2] = pwm_out.duty_c * 12.0f;
        Vofa_Send_JustFloat(vofa_floatdata, 3);
    }
}