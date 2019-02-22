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
#include "power.h"
#include "adc.h"

//四个24v 输出 依次开启 间隔 709us
#define POWER_CTRL_ONE_BY_ONE_TIME 709

void BSP_Pre_Init(void){
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    KEY_Init();
    LED_Init(); 
    BEEP_Init();

    delay_ms(100);
    USART3_Init(115200);

    temperature_ADC_init();

    TIM2_Init();
    SPI5_Init();
    imu_init();

    //24输出控制口 初始化
    power_ctrl_configuration();

    //24v 输出 依次上电
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++){
    power_ctrl_on(i);
    delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }
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
