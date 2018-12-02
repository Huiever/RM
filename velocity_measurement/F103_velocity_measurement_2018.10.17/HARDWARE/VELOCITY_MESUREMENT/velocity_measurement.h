#ifndef __VELOCITY_MEASUREMENT_H
#define __VELOCITY_MEASUREMENT_H
#include "sys.h"

#define  TIMER3_PSC  72             //��Ƶϵ��
#define  IR  PAin(0)                //��������
#define  DISTANCE  0.01228          //17mm����ֱ��-4.72mm���ն�ֱ��


typedef struct Flag
{
	uint8_t across_flag;  //�ӵ���ס�����Ϊtrue
	uint8_t shoot_flag;   //�ӵ����������Ϊtrue
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
extern uint8_t IR_Auto_Check(void); //�Լ�ģ��

#endif
