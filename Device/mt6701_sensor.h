#ifndef __MT6701_SENSOR_H__
#define __MT6701_SENSOR_H__

#include "bsp_spi.h"
#include <stdint.h>

/**
 * @brief MT6701 传感器句柄
 *
 * 每个 MT6701 对应一个实例，挂在某个 bsp_spi_t 上。
 *
 * SSI 帧 (3 bytes):
 *   [15:0]  绝对角度 (0 ~ 65535)
 *   [21:16] 状态字段
 *   [23:22] 0b00
 */
typedef struct {
    const bsp_spi_t *spi; /* SPI 接口指针 */
    float angle_mech;      /* 最近一次角度 (0 ~ 65535) */
    uint8_t status;   /* 最近一次状态字段 */
} mt6701_t;

typedef enum { mt6701_status_success = 0, mt6701_status_comm_error = -1, mt6701_status_crc_error = -2 } mt6701_status_t;

void mt6701_init(mt6701_t *dev, const bsp_spi_t *spidev);
mt6701_status_t mt6701_read_angle(mt6701_t *dev); /* 0 成功, 非 0 失败 */


#endif

