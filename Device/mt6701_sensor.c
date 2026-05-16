#include "mt6701_sensor.h"
#include "stddef.h"



void mt6701_init(mt6701_t *dev, const bsp_spi_t *spidev)
{
    dev->spi        = spidev;
    dev->raw_angle  = 0;
    dev->raw_status = 0;
    dev->spi->init();
}

int mt6701_read_angle(mt6701_t *dev)
{
    uint8_t rx[3] = {0};
    int ret;

    /* SSI 时序：拉低 CS → 等 tCLKFE → 读 3 字节 → 拉高 CS */
    dev->spi->cs_ctrl(true);
    dev->spi->delay_us(3);

    ret = dev->spi->xfer(NULL, rx, 3);
    dev->spi->delay_us(3);
    dev->spi->cs_ctrl(false);

    if (ret != 0) return -1;

    /* 校验帧头：[23:22] 必须为 00 */
    if (rx[2] & 0xC0) return -2;

    dev->raw_angle  = ((uint16_t)rx[0] << 8) | rx[1];
    dev->raw_status = rx[2] & 0x3F;

    return 0;
}