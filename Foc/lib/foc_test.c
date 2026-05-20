#include "foc_test.h"
#include <math.h>
#include <sys/syslimits.h>
#include "bsp_encoder.h"
#include "stm32f407xx.h"
#include "stm32f4xx_it.h"
#include "tim.h"




typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数（未乘 dt）
    float integral;     // 积分累积
    float out_max;      // 输出限幅上限
    float out_min;      // 输出限幅下限
    float dt;           // 控制周期（秒）
} SpeedPI;


Bsp_encoder_t   *g_enc;
FOC_PWM_t       g_FOC_PWM;
SpeedPI         g_speed_pi;
float           g_target_speed_rad_s;



Clarke_ab_t FOC_Clarke(PhaseCurrents_t *pParam)
{
    Clarke_ab_t Output;

    Output.alpha = pParam->a;
    Output.beta = (pParam->a + 2.0f * pParam->b) * _1_DIV_SQRT_3;

    return Output;
}



Park_dq_t FOC_Park(Clarke_ab_t *pParam, float angle_el)
{
    Park_dq_t Output;

    float sin = sinf(angle_el);
    float cos = cosf(angle_el);

    Output.d = pParam->alpha * cos + pParam->beta * sin;
    Output.q = pParam->beta * cos - pParam->alpha * sin;

    return Output;
}



PhaseCurrents_t FOC_InvClarke(Clarke_ab_t *pParam)
{
    PhaseCurrents_t Output;

    Output.a = pParam->alpha;
    Output.b = -0.5f * pParam->alpha + SQRT3_DIV_2 * pParam->beta;
    Output.c = -0.5f * pParam->alpha - SQRT3_DIV_2 * pParam->beta;

    return Output;
}



Clarke_ab_t FOC_InvPark(Park_dq_t *pParam, float angle_el)
{
    Clarke_ab_t Output;

    float sin = sinf(angle_el);
    float cos = cosf(angle_el);

    Output.alpha = pParam->d * cos - pParam->q * sin;
    Output.beta = pParam->d * sin + pParam->q *cos;

    return Output;
}

void FOC_PWM_SetCompare(uint32_t ccr1, uint32_t ccr2, uint32_t ccr3)
{
    htim1.Instance->CCR1 = ccr1;
    htim1.Instance->CCR2 = ccr2;
    htim1.Instance->CCR3 = ccr3;
}


//比较法
uint8_t SectorJudgment(Clarke_ab_t *pAlphabeta)
{
    float A = pAlphabeta->beta;
    float B = SQRT_3 * pAlphabeta->alpha - pAlphabeta->beta;
    float C = -SQRT_3 * pAlphabeta->alpha - pAlphabeta->beta;
    
    uint8_t weight = 0, sector = 0;
    
    if(A > 0)
    {
        weight += 1;
    }
    
    if(B > 0)
    {
        weight += 2;
    }
    
    if(C > 0)
    {
        weight += 4;
    }    
    
    switch(weight)
    {
        case 3:
            sector = 1;
            break;
        
        case 1:
            sector = 2;
            break;
        
        case 5:
            sector = 3;
            break;
        
        case 4:
            sector = 4;
            break;
        
        case 6:
            sector = 5;
            break;
        
        case 2:
            sector = 6;
            break;
        
        default:
            break;
    }
    return sector;
}



//反正切法



static void VectorActionTime(FOC_PWM_t *pFOC_PWM, uint8_t sector, Clarke_ab_t *pAlphabeta)
{
    float T0 = 0, T1 = 0, T2 = 0, sum;            /*0矢量的时间，以及扇区的两个相邻矢量的时间*/
    float vbus = pFOC_PWM->bus_Voltage;
    uint32_t Tpwm = pFOC_PWM->wave_period;
    
    PhaseCurrents_t Tabc_0 = {0};
    PhaseCurrents_t Tabc_1 = {0};

    float tmp = (float) Tpwm*SQRT_3 / vbus;
    
    float x = tmp * pAlphabeta->beta;
    float y = tmp * (SQRT3_DIV_2 * pAlphabeta->alpha + pAlphabeta->beta / 2.0f);
    float z = tmp * (-SQRT3_DIV_2 * pAlphabeta->alpha + pAlphabeta->beta / 2.0f);
    
    switch(sector)
    {
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
    }
    
    //限幅处理
    if(T1 + T2 > Tpwm)
    {
        sum = T1 + T2;
        T1 = T1/sum * Tpwm;
        T2 = T2/sum * Tpwm;
    }
    T0 = Tpwm - T1 - T2;
    
    //相邻矢量和零矢量
    //T0、T1、T2是各个矢量作用总时间，每个不同的时段，输出的矢量是不同的，到时间点就切换
    Tabc_0.a = T0/4.0f;
    Tabc_0.b = Tabc_0.a + T1/2.0f;
    Tabc_0.c = Tabc_0.b + T2/2.0f; 

    //参考三相波形图来看，谁先到比较值就给最小的值
    switch(sector)
    {
        case 1:
            Tabc_1.a = Tabc_0.a;
            Tabc_1.b = Tabc_0.b;
            Tabc_1.c = Tabc_0.c;
            break;
        
        case 2:
            Tabc_1.a = Tabc_0.b;
            Tabc_1.b = Tabc_0.a;
            Tabc_1.c = Tabc_0.c;
            break;  
        

        case 3:
            Tabc_1.a = Tabc_0.a;
            Tabc_1.b = Tabc_0.c;
            Tabc_1.c = Tabc_0.b;
            break;         

        case 4:
            Tabc_1.a = Tabc_0.c;
            Tabc_1.b = Tabc_0.b;
            Tabc_1.c = Tabc_0.a;
            break;

        case 5:
            Tabc_1.a = Tabc_0.b;
            Tabc_1.b = Tabc_0.c;
            Tabc_1.c = Tabc_0.a;
            break;
        
        case 6:
            Tabc_1.a = Tabc_0.a;
            Tabc_1.b = Tabc_0.c;
            Tabc_1.c = Tabc_0.b;
            break;

        default:
            break;
    }
    FOC_PWM_SetCompare(Tabc_1.a, Tabc_1.b, Tabc_1.c);

    pFOC_PWM->Uabc.a = Tabc_1.a / (Tpwm / 2.0f) * vbus;
    pFOC_PWM->Uabc.b = Tabc_1.b / (Tpwm / 2.0f) * vbus;
    pFOC_PWM->Uabc.c = Tabc_1.c / (Tpwm / 2.0f) * vbus;
    
}



void FOC_Run_SVPWM(FOC_PWM_t *pFOC_PWM)
{
    uint8_t sector;
    Clarke_ab_t input = {0};
    
    input = FOC_InvPark(&pFOC_PWM->Uqd, pFOC_PWM->angle_el);
    pFOC_PWM->Ualpha_beta = input;
    
    sector = SectorJudgment(&input);
    VectorActionTime(pFOC_PWM, sector, &input);
    
    
}

void FOC_PWM_Init(void)
{
//    HAL_TIM_Base_Start_IT(&FOC_DRIVER_TIM);

    htim1.Instance->CCR1 = 2625;
    htim1.Instance->CCR2 = 2625;
    htim1.Instance->CCR3 = 2625;

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    g_enc = Bsp_Encoder_Create(7, false, 0.008);

    g_FOC_PWM.bus_Voltage = 12.0f;
    g_FOC_PWM.wave_period = 16000;
}

float Speed_PI_Update(SpeedPI *pPI, float target, float feedback)
{
    float error = target - feedback;
    pPI->integral += error * pPI->dt;

    if (pPI->integral > pPI->out_max)
        pPI->integral = pPI->out_max;
    if (pPI->integral < pPI->out_min)
        pPI->integral = pPI->out_min;

    float output = pPI->Kp * error + pPI->Ki * pPI->integral;
    if (output > pPI->out_max)
        output = pPI->out_max;
    if (output < pPI->out_min)
        output = pPI->out_min;
    return output;
}




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {   
        if (Bsp_Encoder_Update(g_enc) != 0)
        {
            return;
        }

        //机械角度死区
        float speed_fb = Bsp_Encoder_Get_Elec_Speed(g_enc);
        if (fabsf(speed_fb) < 0.5f) 
            speed_fb = 0.0f;

        float ele_angle = Bsp_Encoder_Get_Elec_Angle(g_enc);

        float Vq = Speed_PI_Update(&g_speed_pi, g_target_speed_rad_s, speed_fb);

        g_FOC_PWM.angle_el = ele_angle;
        g_FOC_PWM.Uqd.d    = 0.0f;
        g_FOC_PWM.Uqd.q    = Vq;      
        
        FOC_Run_SVPWM(&g_FOC_PWM);

    }
    
}







