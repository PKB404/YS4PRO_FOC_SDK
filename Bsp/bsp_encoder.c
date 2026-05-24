#include "bsp_encoder.h"
#include "mt6701_sensor.h"
#include <math.h>
#include <stdlib.h>

#define _PI         3.14159265f
#define _2PI        6.283185307f
#define DEG2RAD     (6.283185307f / 360.0f) /* 度转弧度 */

struct bsp_encoder {
    mt6701_t dev;

    uint8_t pole_pairs;
    bool dir_rev;

    float zero_offset;      /* 零位偏移 */

    /* 转速计算相关 (由 update 频率决定) */
    float angle_mech;       /* 机械角度 (rad) */
    float angle_elec;       /* 电角度   (rad), 归一化到 [0, 2π) */
    float speed_mech;       /* 机械角速度 (rad/s) */
    float speed_elec;       /* 电角速度   (rad/s) */

    float last_angle;       /* 上一拍机械角度 (rad), 算速度用 */
    float update_period;    /* 秒, 由外部设定 */
    float speed_filter;     /* 低通滤波系数 (0~1), 0=关闭 */
};

/* ---------- create / destroy ---------- */

Bsp_encoder_t *Bsp_Encoder_Create(uint8_t pole_pairs, bool dir_rev, float period) {
    Bsp_encoder_t *enc = calloc(1, sizeof(Bsp_encoder_t));
    if (!enc)
        return NULL;

    mt6701_init(&enc->dev, &g_spi_mt6701);

    enc->pole_pairs = pole_pairs;
    enc->dir_rev = dir_rev;
    enc->update_period = period;
    enc->speed_filter = 0.3f;
    enc->zero_offset = 0.0f;

    /* 首次读取, 初始化 last_raw 避免速度跳变 */
    mt6701_read_angle(&enc->dev);
    enc->last_angle = enc->dev.angle_mech * DEG2RAD;

    return enc;
}

void Bsp_Encoder_Destroy(Bsp_encoder_t *enc) { 
    free(enc); 
}

/* ---------- 数据更新 ---------- */

int Bsp_Encoder_Update(Bsp_encoder_t *enc) {
    if (mt6701_read_angle(&enc->dev) != mt6701_status_success)
        return -1;

    float mech_deg = enc->dev.angle_mech;
    mech_deg -= enc->zero_offset;
    if (mech_deg < 0.0f)
        mech_deg += 360.0f;

    enc->angle_mech = mech_deg * DEG2RAD;

    if (enc->dir_rev){
        enc->angle_mech = _2PI - enc->angle_mech;
        if (enc->angle_mech >= _2PI)
            enc->angle_mech = 0.0f;
    }    

    float delta = enc->angle_mech - enc->last_angle;
    if (delta > _PI)
        delta -= _2PI;
    else if (delta < -_PI)
        delta += _2PI;

    // /*一阶低通滤波*/
    // float raw_speed = delta / enc->update_period;    /* 瞬时速度 rad/s */
    // if (enc->speed_filter > 0.0f && enc->speed_filter < 1.0f){
    //     enc->speed_mech = enc->speed_filter * raw_speed \
    //                     + (1.0f - enc->speed_filter) * enc->speed_mech;
    // }
    // else {
    //     enc->speed_mech = raw_speed;
    // }

    float elec = enc->angle_mech * enc->pole_pairs;
    enc->angle_elec = fmodf(elec, _2PI);                        /* 归一化到 [0, 2π) */
    enc->speed_elec = enc->speed_mech * (float)enc->pole_pairs;

    enc->last_angle = enc->angle_mech;

    return 0;
}

float Bsp_Encoder_Get_Mech_Angle(Bsp_encoder_t *enc) {
    return enc->angle_mech; 
}

float Bsp_Encoder_Get_Elec_Angle(Bsp_encoder_t *enc) { 
    return enc->angle_elec; 
}

float Bsp_Encoder_Get_Mech_Speed(Bsp_encoder_t *enc) { 
    return enc->speed_mech; 
}

float Bsp_Encoder_Get_Elec_Speed(Bsp_encoder_t *enc) { 
    return enc->speed_elec; 
}

/* ========== 零位校准 ========== */

void Bsp_Encoder_Set_Zero(Bsp_encoder_t *enc) { 
    enc->zero_offset = enc->dev.angle_mech; 
}

/* ========== 滤波系数 ========== */

void Bsp_Encoder_Set_Speed_Filter(Bsp_encoder_t *enc, float coeff) {
    if (coeff < 0.0f)
        coeff = 0.0f;
    if (coeff > 1.0f)
        coeff = 1.0f;
    enc->speed_filter = coeff;
}

void Bsp_Encoder_Set_Zero_Offset(Bsp_encoder_t *enc, float offset_deg)
{
    if (enc == NULL) return;
    enc->zero_offset = offset_deg;
}



