#include "can1.h"
#include "can_bus_task.h"

void CAN1_Init(void){
  CAN_InitTypeDef        can;
  CAN_FilterInitTypeDef  can_filter;
  GPIO_InitTypeDef       gpio;
  NVIC_InitTypeDef       nvic;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);
  gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
  gpio.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOD, &gpio);
  nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 1;
  nvic.NVIC_IRQChannelSubPriority = 1;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);
  nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 0;
  nvic.NVIC_IRQChannelSubPriority = 1;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);    
  CAN_DeInit(CAN1);
  CAN_StructInit(&can);
  can.CAN_TTCM = DISABLE;
  can.CAN_ABOM = DISABLE;
  can.CAN_AWUM = DISABLE;
  can.CAN_NART = DISABLE;
  can.CAN_RFLM = DISABLE;
  can.CAN_TXFP = ENABLE;
  can.CAN_Mode = CAN_Mode_Normal;
  can.CAN_SJW  = CAN_SJW_1tq;
  can.CAN_BS1 = CAN_BS1_9tq;
  can.CAN_BS2 = CAN_BS2_4tq;
  can.CAN_Prescaler = 3;
  CAN_Init(CAN1, &can);
	can_filter.CAN_FilterNumber=0;
	can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
	can_filter.CAN_FilterIdHigh=0x0000;
	can_filter.CAN_FilterIdLow=0x0000;
	can_filter.CAN_FilterMaskIdHigh=0x0000;
	can_filter.CAN_FilterMaskIdLow=0x0000;
	can_filter.CAN_FilterFIFOAssignment=0;
	can_filter.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&can_filter);
  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
  CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 
}

void CAN1_TX_IRQHandler(void){
	if(CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET){
	  CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
	}
}

void CAN1_RX0_IRQHandler(void){
	CanRxMsg rx_message;
  if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET){
    CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
	  CAN_ClearFlag(CAN1, CAN_FLAG_FF0); 
		CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
   	CanReceiveMsgProcess(&rx_message);
  }
}
