#ifndef _GUN_H_
#define _GUN_H_

#ifndef FRICTION_WHEEL
#define FRICTION_WHEEL
#endif

#if defined(FRICTION_WHEEL)

#define PWM1  TIM5->CCR1
#define PWM2  TIM5->CCR2

#define InitOrStopFrictionWheel() \
        PWM1 = 1000;        \
        PWM2 = 1000;
#define SetFrictionWheelSpeed_1(x)    PWM1 = x;
#define SetFrictionWheelSpeed_2(x)    PWM2 = x;

#endif

void Gun_Configuration(void);
void Gun_Init(void);

#endif
