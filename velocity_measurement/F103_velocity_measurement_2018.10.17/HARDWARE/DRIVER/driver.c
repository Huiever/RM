#include "driver.h"

void BSP_INIT(void)
{
	delay_init();	    	 //��ʱ������ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// �����ж����ȼ�����2
	uart_init(115200);
	LED_Init();		  	//��ʼ����LED���ӵ�Ӳ���ӿ�
	IR_Init(); //����ų�ʼ��
	EXTIX_Falling_Init();     //�ⲿ�жϳ�ʼ��
	TIM3_Int_Init(65535,TIMER3_PSC-1);//1Mhz�ļ���Ƶ��
	OLED_Init();			        //��ʼ��OLED  
	OLED_Clear(); 
}
