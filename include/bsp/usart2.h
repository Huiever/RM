#ifndef __USART2_H
#define __USART2_H

#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h"

#define USART_REC_LEN  200
#define EN_USART2_RX 	 1
	  	
extern u8  USART_RX_BUF_2[USART_REC_LEN];
extern u16 USART_RX_STA_2;

void USART2_Init(u32 bound);
void USART2_SendChar(unsigned char b);

#endif
