#ifndef _USART6_H_
#define _USART6_H_

#include "sys.h"

void USART6_Configuration(u32 bound);
void UART6_PrintCh(uint8_t ch);
void USART6_Init(u32 bound);
void UART6_Print_Status(int8_t const pitch_angle);
void USART6_Print(uint8_t* ch, int len);

#endif
