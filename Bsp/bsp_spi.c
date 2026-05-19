#include "bsp_spi.h"
#include "board.h"
#include "spi.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"

#define SPI1_CS_Pin GPIO_PIN_15
#define SPI1_CS_GPIO_Port GPIOG
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_4
#define SPI1_MISO_GPIO_Port GPIOB
#define SPI1_MOSI_Pin GPIO_PIN_5
#define SPI1_MOSI_GPIO_Port GPIOB

/*==================== MT6701 编码器 SPI1 ====================*/

static void _mt6701_init(void) {
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    GPIO_InitTypeDef spi1_gpio = {
        .Pin = SPI1_CS_Pin,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
    };
    // SPI1_CS[Soft CS]
    HAL_GPIO_Init(SPI1_CS_GPIO_Port, &spi1_gpio);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); // 初始拉高

    spi1_gpio.Pin = SPI1_SCK_Pin;
    spi1_gpio.Mode = GPIO_MODE_AF_PP;
    spi1_gpio.Pull = GPIO_NOPULL;
    spi1_gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    spi1_gpio.Alternate = GPIO_AF5_SPI1;

    // SPI1_SCK
    HAL_GPIO_Init(SPI1_SCK_GPIO_Port, &spi1_gpio);

    // SPI1_MISO | SPI1_MISO
    spi1_gpio.Pin = SPI1_MISO_Pin | SPI1_MOSI_Pin;
    HAL_GPIO_Init(SPI1_MISO_GPIO_Port, &spi1_gpio);

    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    HAL_SPI_Init(&hspi1);
}

static void _mt6701_deinit(void) { HAL_SPI_DeInit(&hspi1); }

static void _mt6701_cs_ctrl(bool active) {
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, active ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void _mt6701_delay_us(uint32_t us) { Delay_us(us); }

static int _mt6701_xfer(uint8_t *rx, uint16_t len) {
    uint8_t dummy[3] = {0x00, 0x00, 0x00};
    return (HAL_SPI_TransmitReceive(&hspi1, dummy, rx, len, 10) == HAL_OK) ? 0 : -1;
}

/*---------- 导出 ----------*/
const bsp_spi_t g_spi_mt6701 = {
    .init = _mt6701_init,
    .deinit = _mt6701_deinit,
    .cs_ctrl = _mt6701_cs_ctrl,
    .delay_us = _mt6701_delay_us,
    .xfer = _mt6701_xfer,
};
