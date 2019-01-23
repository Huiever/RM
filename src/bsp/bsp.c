#include "bsp.h"
#include "sys.h"
#include "control_task.h"
#include "remote_task.h"
#include "judge_task.h"
#include "gun.h"
#include "laser.h"
#include "usart6.h"
#include "key.h"
#include "led.h"
#include "beep.h"
#include "timer.h"
#include "spi.h"
#include "imu.h"
#include "can1.h"
#include "can2.h"
#include "dbus.h" 
#include "delay.h"
#include "usart3.h"
#include "flags.h"

void BSP_Pre_Init(void){
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	KEY_Init();
	LED_Init(); 
	BEEP_Init();
	
	delay_ms(100);
	USART3_Init(115200);
	
	TIM2_Init();
	SPI5_Init();
	imu_init();
}

void BSP_Init(void)
{
	ControtTaskInit();
	RemoteTaskInit();
	Flags_Init();
	
	Gun_Init();
	Laser_Init();
	delay_ms(100);
	Judge_Init();
	USART6_Init(115200);
	
	TIM6_Init();
	
	CAN1_Init();
	CAN2_Init();
	Dbus_Init();
	
	TIM6_Start();
}
