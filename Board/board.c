#include "board.h"
#include "stm32f4xx_hal.h"

/**
 * @brief 初始化 DWT 时钟计数器
 */
void Board_DWT_Init(void) {
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

/**
 * @brief 微秒级延时（基于 DWT 时钟周期，中断安全）
 */
void Delay_us(uint32_t nus) {
    // 防止忘记调用 board_delay_init
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        Board_DWT_Init();
    }
    uint32_t start = DWT->CYCCNT;
    uint32_t delay = nus * (SystemCoreClock / 1000000U);
    // 利用无符号回绕自动处理计数器溢出
    while ((DWT->CYCCNT - start) < delay)
        ;
}

/**
 * @brief 毫秒级延时（基于 DWT，通过多次 1ms 微秒延时避免大数溢出）
 */
void Delay_ms(uint32_t nms) {
    while (nms--) {
        Delay_us(1000U);
    }
}
