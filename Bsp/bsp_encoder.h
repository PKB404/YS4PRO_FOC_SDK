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
typedef struct bsp_encoder bsp_encoder_t;

/**
 * @param spidev     已初始化的 SPI 设备指针（例: &g_spi_mt6701）
 * @param pole_pairs 电机极对数
 * @param dir_rev    是否反转方向
 * @return 成功返回句柄, 失败返回 NULL
 */
bsp_encoder_t *bsp_encoder_create(uint8_t pole_pairs,
                                  bool dir_rev);
void bsp_encoder_destroy(bsp_encoder_t *enc);

int   bsp_encoder_update(bsp_encoder_t *enc);       /* 读一次传感器 */

float bsp_encoder_get_mech_angle(const bsp_encoder_t *enc);   /* rad, [0, 2π) */
float bsp_encoder_get_elec_angle(const bsp_encoder_t *enc);   /* rad, [0, 2π) */
float bsp_encoder_get_speed(const bsp_encoder_t *enc);        /* rad/s */

void     bsp_encoder_set_zero(bsp_encoder_t *enc);            /* 当前角度设为零位 */
uint16_t bsp_encoder_get_raw(const bsp_encoder_t *enc);       /* 原始值, 调试用 */





#endif
