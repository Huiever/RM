#ifndef __VELOCITY_MEASUREMENT_H
#define __VELOCITY_MEASUREMENT_H
#include "sys.h"

#define  TIMER3_PSC  72             //分频系数
#define  IR  PAin(0)                //光电管引脚
#define  DISTANCE  0.01228          //17mm弹丸直径-4.72mm接收端直径


typedef struct Flag
{
	uint8_t across_flag;  //子弹遮住光电门为true
	uint8_t shoot_flag;   //子弹经过光电门为true
}Flag;

#define V_FlAG_DAFAULT \
{ \
		.across_flag = 0, \
		.shoot_flag = 0, \
}

extern unsigned int counter;
extern Flag V_Flag;

extern double velocity_measurement(void);
extern void IR_Init(void);
extern void EXTIX_Falling_Init(void);
extern uint8_t IR_Auto_Check(void); //自检模块

#endif
