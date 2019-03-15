#include "dbus.h"
#include "delay.h"
#include "remote_task.h"

static uint8_t Dbus_rx_buffer[25];

void Dbus_Init(){
  GPIO_InitTypeDef    GPIO_InitStructure;
  USART_InitTypeDef    USART_InitStructure;
  NVIC_InitTypeDef    NVIC_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource7, GPIO_AF_USART1);
  USART_DeInit(USART1);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  USART_InitStructure.USART_BaudRate = 100000;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_Even;
  USART_InitStructure.USART_Mode = USART_Mode_Rx;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART1, &USART_InitStructure);
  USART_Cmd(USART1, ENABLE);
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void USART1_IRQHandler(void){
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){
        DMA_InitTypeDef        dma;
        USART_InitTypeDef    USART_InitStructure;
        NVIC_InitTypeDef    nvic;
        USART_DeInit(USART1);
        delay_ms(3);
        RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA2, ENABLE);
        USART_InitStructure.USART_BaudRate = 100000;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_Even;
        USART_InitStructure.USART_Mode = USART_Mode_Rx;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(USART1, &USART_InitStructure);
        USART_Cmd(USART1, ENABLE);
        USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
        nvic.NVIC_IRQChannel = DMA2_Stream2_IRQn;
        nvic.NVIC_IRQChannelPreemptionPriority = 0;
        nvic.NVIC_IRQChannelSubPriority = 1;
        nvic.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&nvic);
        DMA_DeInit(DMA2_Stream2);
        dma.DMA_Channel = DMA_Channel_4;
        dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
        dma.DMA_Memory0BaseAddr = (uint32_t)Dbus_rx_buffer;
        dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
        dma.DMA_BufferSize = 18;
        dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
        dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        dma.DMA_Mode = DMA_Mode_Circular;
        dma.DMA_Priority = DMA_Priority_VeryHigh;
        dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
        dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        dma.DMA_MemoryBurst = DMA_Mode_Normal;
        dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA2_Stream2,&dma);
        DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,ENABLE);
        DMA_Cmd(DMA2_Stream2,ENABLE);
  }
}

void DMA2_Stream2_IRQHandler(void){
  if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2)){
    DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
    DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
    RemoteDataProcess(Dbus_rx_buffer);
  }
}
