#ifndef __BSP_ENCODER_H__
#define __BSP_ENCODER_H__

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 编码器抽象句柄（不透明指针，换芯片只改 .c 内部）
 *
 * 应用层通过此接口获取：
 *  - 机械角度 / 电角度 (rad)
 *  - 转速 (rad/s)
 *  - 零位校准
 */
typedef struct bsp_encoder Bsp_encoder_t;

/**
 * @param spidev     已初始化的 SPI 设备指针（例: &g_spi_mt6701）
 * @param pole_pairs 电机极对数
 * @param dir_rev    是否反转方向
 * @return 成功返回句柄, 失败返回 NULL
 */
Bsp_encoder_t *Bsp_Encoder_Create(uint8_t pole_pairs, bool dir_rev, float period);

void Bsp_Encoder_Destroy(Bsp_encoder_t *enc);

int Bsp_Encoder_Update(Bsp_encoder_t *enc); /* 读一次传感器 */

float Bsp_Encoder_Get_Mech_Angle(Bsp_encoder_t *enc); /* 机械角度 (rad)  */
float Bsp_Encoder_Get_Elec_Angle(Bsp_encoder_t *enc); /* 电角度   (rad)  */
float Bsp_Encoder_Get_Mech_Speed(Bsp_encoder_t *enc); /* 机械角速度 (rad/s) */
float Bsp_Encoder_Get_Elec_Speed(Bsp_encoder_t *enc); /* 电角速度   (rad/s) */

/** 设置当前角度为零位 */
void Bsp_Encoder_Set_Zero(Bsp_encoder_t *enc);

/** 设置速度滤波系数 (0=不过滤, 接近1=强滤波) */
void Bsp_Encoder_Set_Speed_Filter(Bsp_encoder_t *enc, float coeff);

/** 设置零点偏移（单位：度），校准用 */
void Bsp_Encoder_Set_Zero_Offset(Bsp_encoder_t *enc, float offset_deg);


#endif
