#include "driver.h"

void BSP_INIT(void)
{
	delay_init();	    	 //延时函数初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	uart_init(115200);
	LED_Init();		  	//初始化与LED连接的硬件接口
	IR_Init(); //光电门初始化
	EXTIX_Falling_Init();     //外部中断初始化
	TIM3_Int_Init(65535,TIMER3_PSC-1);//1Mhz的计数频率
	OLED_Init();			        //初始化OLED  
	OLED_Clear(); 
}
