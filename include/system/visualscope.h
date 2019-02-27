#ifndef __VISUALSCOPE_H__
#define __VISUALSCOPE_H__

#include "stm32f4xx.h"

void RS232_VisualScope(USART_TypeDef* USARTx, u8*, u16);
void VisualScope(USART_TypeDef* USARTx,int16_t CH1,int16_t CH2,int16_t CH3,int16_t CH4);
//void usart2_niming_report(u8 fun,u8*data,u8 len);
//void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz);
//void usart2_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw);

#endif
