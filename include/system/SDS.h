/**************************************************************************
Module Name: Serial Digital Scope Module
Module Date:  2013/4/2
Module Auth:  XiaoGo_O Modulated
Description:   Serial Digital Scope For XS128
Others:      Modulate for S12XS128 112PINS.
           DEBUG SUCCESS IN 2013/4/7
           in 32M Bus Speed.
 Revision History:

WARN!
SRS.c depend on USART
please ensure that uart_putchar() has been implemented
Ensure that SCI has been configured in 9600bps mode.
**************************************************************************/
#ifndef SDS_outputdata_H
#define SDS_outputdata_H
/*------------------------------------Extern Variables------------------------------------*/
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);
void OCS_displayData(float CH1,float CH2,float CH3,float CH4);
//void OutPut_Data(void);
#endif  
