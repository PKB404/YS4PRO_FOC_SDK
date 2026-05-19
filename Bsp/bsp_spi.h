#ifndef __BSP_SPI_H__
#define __BSP_SPI_H__

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief SPI 设备操作接口（每个 SPI 设备一个实例）
 *
 * 换 MCU 时只改 bsp_spi.c 里的实现，应用层无感
 */
typedef struct {
    void (*init)(void);                     // 初始化 SPI 外设
    void (*deinit)(void);                   // 反初始化
    void (*cs_ctrl)(bool active);           // 片选控制 (true=拉低, false=拉高)
    void (*delay_us)(uint32_t us);          // 微秒延时
    int (*xfer)(uint8_t *rx, uint16_t len); // 全双工传输，返回 0 成功
} bsp_spi_t;

/* 声明你项目里的 SPI 设备实例（全局唯一，应用层直接用） */
extern const bsp_spi_t g_spi_mt6701; // 编码器

#endif
