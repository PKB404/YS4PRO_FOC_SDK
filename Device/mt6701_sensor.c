#include "mt6701_sensor.h"
#include "stddef.h"
#include <stdint.h>

static uint8_t mt6701_crc6(uint8_t *pData, uint16_t length) {
    uint8_t count;
    uint8_t crc = 0;

    while (length--) {
        crc ^= *pData++;
        for (count = 6; count > 0; --count) {
            if (crc & 0x20)
                crc = (crc << 1) ^ 0x03;
            else
                crc = (crc << 1);
        }
    }
    return (crc & 0x3f);
}

void mt6701_init(mt6701_t *dev, const bsp_spi_t *spidev) {
    dev->spi = spidev;
    dev->angle_mech = 0;
    dev->status = 0;
    dev->spi->init();
}

mt6701_status_t mt6701_read_angle(mt6701_t *dev) {
    uint8_t rx[3] = {0};
    uint32_t raw_data = 0;
    uint8_t crc_cal[3] = {0x00};
    int ret;

    /* SSI 时序：拉低 CS → 等 tCLKFE → 读 3 字节 → 拉高 CS */
    dev->spi->cs_ctrl(true);
    ret = dev->spi->xfer(rx, 3);
    dev->spi->cs_ctrl(false);

    if (ret != 0)
        return mt6701_status_comm_error;

    raw_data = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | rx[2];
    crc_cal[0] = raw_data >> 18;
    crc_cal[1] = raw_data >> 12;
    crc_cal[2] = raw_data >> 6;

    if (mt6701_crc6(crc_cal, 3) != ((uint8_t)raw_data & 0x3F))
        return mt6701_status_crc_error;

    dev->angle_mech = (raw_data >> 10) * 360.0f / 16384.0f;
    dev->status = (raw_data & 0x3C0) >> 6;

    return mt6701_status_success;
}