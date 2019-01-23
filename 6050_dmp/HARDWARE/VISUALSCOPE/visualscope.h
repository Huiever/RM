#ifndef __VISUALSCOPE_H__
#define __VISUALSCOPE_H__

#include "sys.h"
#include "usart.h"

//通过串口发送VisualScope识别的数据
void RS232_VisualScope	(USART_TypeDef* USARTx, u8*, u16 );

//通过串口发送VisualScope识别的数据
void VisualScope		(USART_TypeDef* USARTx,int16_t CH1,int16_t CH2,int16_t CH3,int16_t CH4);

#endif
