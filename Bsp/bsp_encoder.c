#include "bsp_encoder.h"
#include "mt6701_sensor.h"
#include <stdlib.h>
#include <math.h>

#define _2PI  6.283185307f
#define SCALE (6.283185307f / 65536.0f)     /* 2π / 65536 */

struct bsp_encoder {
    mt6701_t dev;

    uint8_t  pole_pairs;
    bool     dir_rev;

    uint16_t zero_offset;   /* 零位偏移 */
    uint16_t last_raw;      /* 上一拍角度, 算速度用 */

    /* 转速计算相关 (由 update 频率决定) */
    float    speed;          /* rad/s */
    float    update_period;  /* 秒, 由外部设定 */
    float    speed_filter;   /* 低通滤波系数 (0~1), 0=关闭 */
};

/* ---------- create / destroy ---------- */

bsp_encoder_t *bsp_encoder_create(uint8_t pole_pairs,
                                  bool dir_rev)
{
    bsp_encoder_t *enc = calloc(1, sizeof(bsp_encoder_t));
    if (!enc) 
        return NULL;

    mt6701_init(&enc->dev, &g_spi_mt6701);

    enc->pole_pairs    = pole_pairs;
    enc->dir_rev       = dir_rev;
    enc->update_period = 0.0001f;     /* 默认 10 kHz */
    enc->speed_filter  = 0.3f;

    /* 首次读取, 初始化 last_raw 避免速度跳变 */
    mt6701_read_angle(&enc->dev);
    enc->last_raw = enc->dev.raw_angle;

    return enc;
}

void bsp_encoder_destroy(bsp_encoder_t *enc)
{
    free(enc);
}

/* ---------- 数据更新 ---------- */

int bsp_encoder_update(bsp_encoder_t *enc)
{
    if (mt6701_read_angle(&enc->dev) != 0)
        return -1;

    uint16_t cur = enc->dev.raw_angle;

    /* 角度差值 (自动处理 0 ↔ 65535 跳变) */
    int16_t delta = (int16_t)(cur - enc->last_raw);
    enc->last_raw = cur;

    /* 瞬时速度 = delta * (2π/65536) / update_period */
    float inst_speed = (float)delta * SCALE / enc->update_period;

    /* 一阶低通滤波 */
    enc->speed = enc->speed_filter * inst_speed
               + (1.0f - enc->speed_filter) * enc->speed;

    if (enc->dir_rev)
        enc->speed = -enc->speed;

    return 0;
}

/* ---------- 读取 ---------- */

float bsp_encoder_get_mech_angle(const bsp_encoder_t *enc)
{
    uint16_t raw = enc->dev.raw_angle - enc->zero_offset;
    float angle = (float)raw * SCALE;
    return enc->dir_rev ? (_2PI - angle) : angle;
}

float bsp_encoder_get_elec_angle(const bsp_encoder_t *enc)
{
    float mech = bsp_encoder_get_mech_angle(enc);
    float elec = mech * enc->pole_pairs;
    return fmodf(elec, _2PI);
}

float bsp_encoder_get_speed(const bsp_encoder_t *enc)
{
    return enc->speed;
}

uint16_t bsp_encoder_get_raw(const bsp_encoder_t *enc)
{
    return enc->dev.raw_angle;
}

void bsp_encoder_set_zero(bsp_encoder_t *enc)
{
    enc->zero_offset = enc->dev.raw_angle;
}