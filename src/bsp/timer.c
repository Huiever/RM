  #include "timer.h"
#include "control_task.h"

void TIM2_Init(void){
  TIM_TimeBaseInitTypeDef tim;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
  tim.TIM_Period = 0xFFFFFFFF;
  tim.TIM_Prescaler = 84 - 1; 
  tim.TIM_ClockDivision = TIM_CKD_DIV1;	
  tim.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_ARRPreloadConfig(TIM2, ENABLE);	
  TIM_TimeBaseInit(TIM2, &tim);
  TIM_Cmd(TIM2,ENABLE);
}

void TIM3_Init(uint16_t arr, uint16_t psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM3, DISABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_TimeBaseInitStructure.TIM_Period = arr - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    /* TIM3 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);

    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM3, ENABLE);

    TIM_Cmd(TIM3, ENABLE);
}


void TIM4_Init(void){
  TIM_TimeBaseInitTypeDef tim;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
  tim.TIM_Period = 0xFFFFFFFF;
  tim.TIM_Prescaler = 168-1; 
  tim.TIM_ClockDivision = TIM_CKD_DIV1;
  tim.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_ARRPreloadConfig(TIM4, ENABLE);
  TIM_TimeBaseInit(TIM4, &tim);
  TIM_ARRPreloadConfig(TIM4, ENABLE);	
  TIM_PrescalerConfig(TIM4, 0, TIM_PSCReloadMode_Update);
  TIM_UpdateDisableConfig(TIM4, ENABLE);
  TIM_Cmd(TIM4,ENABLE);
}

void TIM6_Init(void){
  TIM_TimeBaseInitTypeDef tim;
  NVIC_InitTypeDef nvic;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
  nvic.NVIC_IRQChannel = TIM6_DAC_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 1;
  nvic.NVIC_IRQChannelSubPriority = 0;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);
  tim.TIM_Prescaler = 84-1;
  tim.TIM_CounterMode = TIM_CounterMode_Up;
  tim.TIM_ClockDivision = TIM_CKD_DIV1;
  tim.TIM_Period = 1500;
  TIM_TimeBaseInit(TIM6,&tim);
}

void TIM8_Init(void){
  TIM_TimeBaseInitTypeDef tim;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
  tim.TIM_Period = 0xFFFFFFFF;
  tim.TIM_Prescaler = 168-1;
  tim.TIM_ClockDivision = TIM_CKD_DIV1;
  tim.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_ARRPreloadConfig(TIM8, ENABLE);
  TIM_TimeBaseInit(TIM8, &tim);
  TIM_ARRPreloadConfig(TIM8, ENABLE);
  TIM_PrescalerConfig(TIM8, 0, TIM_PSCReloadMode_Update);
  TIM_UpdateDisableConfig(TIM8, ENABLE);
  TIM_Cmd(TIM8,ENABLE);
}

uint32_t Get_Time_Micros(void){
  return (TIM2->CNT)/1000;
}

void TIM6_Start(void){
  TIM_Cmd(TIM6, ENABLE);
  TIM_ITConfig(TIM6, TIM_IT_Update,ENABLE);
  TIM_ClearFlag(TIM6, TIM_FLAG_Update);
}

void TIM2_IRQHandler(void){
  if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET){
    TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  }
}

void TIM6_DAC_IRQHandler(void){
  if(TIM_GetITStatus(TIM6,TIM_IT_Update) != RESET){
    TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);
    Control_Task();
  }
}
