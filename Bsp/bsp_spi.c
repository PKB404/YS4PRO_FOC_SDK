#include "bsp_spi.h"
#include "spi.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_gpio.h"

#define SPI1_CS_Pin GPIO_PIN_15
#define SPI1_CS_GPIO_Port GPIOG
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_4
#define SPI1_MISO_GPIO_Port GPIOB

/*==================== MT6701 编码器 SPI1 ====================*/

static void _mt6701_init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    GPIO_InitTypeDef spi1_gpio = {
        .Pin = SPI1_SCK_Pin,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = GPIO_AF5_SPI1,
    };
    // SPI1_SCK
    HAL_GPIO_Init(SPI1_SCK_GPIO_Port, &spi1_gpio);

    // SPI1_MISO
    spi1_gpio.Pin = SPI1_MISO_Pin;
    HAL_GPIO_Init(SPI1_MISO_GPIO_Port, &spi1_gpio);

    // SPI1_CS[Soft CS]
    spi1_gpio.Pin = SPI1_CS_Pin;
    spi1_gpio.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(SPI1_CS_GPIO_Port, &spi1_gpio);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); // 初始拉高

    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    HAL_SPI_Init(&hspi1);
}

static void _mt6701_cs_ctrl(bool active)
{
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, active ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void _mt6701_delay_us(uint32_t us)
{
    // DWT 实现，中断安全
    uint32_t start = DWT->CYCCNT;
    uint32_t delay = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < delay);
}










