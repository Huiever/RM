#ifndef _TIMER_H_
#define _TIMER_H_

#include "sys.h"

void TIM2_Init(void);
void TIM4_Init(void);
void TIM6_Init(void);
void TIM8_Init(void);
void TIM6_Start(void);
uint32_t Get_Time_Micros(void);

#endif
