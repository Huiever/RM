#include "usart2.h"
#if EN_USART2_RX    
u8 USART_RX_BUF_2[USART_REC_LEN];
u16 USART_RX_STA_2=0;
 #if 1
#pragma import(__use_no_semihosting)              
struct __FILE {
    int handle; 
}; 

FILE __stdout; 
void _sys_exit(int x){
    x = x; 
}

int fputc(int ch, FILE *f){
    while((USART2->SR&0X40)==0); 
    USART2->DR = (u8) ch;  
    return ch;
}
#endif
void USART2_Init(u32 bound){
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD,&GPIO_InitStructure);
    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);
    USART_Cmd(USART2, ENABLE);
#if EN_USART2_RX    
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
}

void USART2_IRQHandler(void){
    u8 Res;
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET){
        Res =USART_ReceiveData(USART2);
        if((USART_RX_STA_2&0x8000)==0){
            if(USART_RX_STA_2&0x4000){
                if(Res!=0x0a)USART_RX_STA_2=0;
                else USART_RX_STA_2|=0x8000;
            }
            else{    
                if(Res==0x0d)
                    USART_RX_STA_2|=0x4000;
                else{
                    USART_RX_BUF_2[USART_RX_STA_2&0X3FFF]=Res ;
                    USART_RX_STA_2++;
                    if(USART_RX_STA_2>(USART_REC_LEN-1))
                        USART_RX_STA_2=0; 
                }         
            }
        }
    }
}
#endif    

void USART2_SendChar(unsigned char b){
    while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)== RESET)
        ;
  USART_SendData(USART2,b);
}
