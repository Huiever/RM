#ifndef __USART3_H
#define __USART3_H

#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h"

#define USART_REC_LEN  200
#define EN_USART3_RX 	 1
	  	
extern u8  USART_RX_BUF[USART_REC_LEN];
extern u16 USART_RX_STA;

void USART3_Init(u32 bound);
void USART3_SendChar(unsigned char b);

#endif


