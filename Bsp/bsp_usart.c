#include "bsp_usart.h"
#include "usart.h"

#ifdef __GNUC__
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 50);
    return ch;
}
#else
int fputc(int ch, FILE *stream) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 50);
    return ch;
}
#endif

void USART1_SendBuff(uint8_t *pbuff, uint32_t len) {
    uint32_t count;
    for (count = 0; count < len; count++) {
        printf("%02x", pbuff[count]);
    }
    printf("\r\n");
}

// /* 最大通道数 */
// #define VOFA_MAX_CHANNELS 8

// /* JustFloat协议帧尾(NaN值: 0x00,0x00,0x80,0x7F) */
// #define VOFA_JUSTFLOAT_TAIL 0x7F800000U /* NaN作为帧尾标记 */

// static uint8_t g_tx_buf[VOFA_MAX_CHANNELS * sizeof(float) + sizeof(float)];

// int Vofa_Send_JustFloat(const float *values, uint8_t count) {
//     uint32_t tail_value;
//     uint32_t offset = 0;

//     if (!values || count > VOFA_MAX_CHANNELS) {
//         return -1;
//     }

//     memcpy(g_tx_buf, values, count * sizeof(float));
//     offset += count * sizeof(float);

//     tail_value = VOFA_JUSTFLOAT_TAIL;
//     memcpy(&g_tx_buf[offset], &tail_value, sizeof(float));
//     offset += sizeof(float);

//     HAL_UART_Transmit(&huart1, g_tx_buf, offset, 50);
//     return 0;
// }
