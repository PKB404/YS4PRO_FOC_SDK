#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include <stdbool.h>
#include <stdint.h>

#define DEBUG_PRINTF

#ifdef DEBUG_PRINTF
#define DEBUG_Log(format, arg...) printf(format, ##arg)
#define DEBUG_Send USART1_SendBuff
#else
#define DEBUG_Log(format, arg...) ((void)0)
#define DEBUG_Send ((void)0)
#endif

void USART1_SendBuff(uint8_t *pbuff, uint32_t len);

// int Vofa_Send_JustFloat(const float *values, uint8_t count);

#endif
