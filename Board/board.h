#ifndef __BOARD_H__
#define __BOARD_H__

#include <stdint.h>

void Board_DWT_Init(void);

void Delay_us(uint32_t nus);

void Delay_ms(uint32_t nms);

#endif
