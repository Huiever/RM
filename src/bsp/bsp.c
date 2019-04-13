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
#include "usart2.h"
#include "usart3.h"
#include "flags.h"
#include "power.h"
#include "adc.h"

void BSP_Pre_Init(void){
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    KEY_Init();
    LED_Init(); 
    BEEP_Init();

    delay_ms(100);
    USART2_Init(115200);
    USART3_Init(115200);
    USART6_Init(115200);
    
    SPI5_Init();

    power_init();  //24V电源依次开启
}

void BSP_Init(void)
{
    ControtTaskInit();
    RemoteTaskInit();
    Flags_Init();

    Gun_Init();
#if Calibrate_Snail_ESC == 1       //油门校准，零油门--1ms高电平 满油门--2ms高电平
    SetFrictionWheelSpeed(2000);        
    delay_ms(3000);
    SetFrictionWheelSpeed(1000);
    delay_ms(3000);
#endif
#if Calibrate_Snail_ESC == 2       //转向切换，需要手动转动摩擦轮改变摩擦轮的方向
    SetFrictionWheelSpeed(2000);
    delay_ms(6000);
    SetFrictionWheelSpeed(1000);
    delay_ms(3000);
#endif
    Laser_Init();
    delay_ms(100);
    imu_init();
//    Judge_Init();

    TIM6_Init();
    CAN1_Init();
    CAN2_Init();
    Dbus_Init();

    TIM6_Start();
}
