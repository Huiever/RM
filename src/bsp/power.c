#include "power.h"
#include "sys.h"
#include "delay.h"

void power_ctrl_configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOH, &GPIO_InitStructure);

    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        power_ctrl_off(i);
    }
}
void power_ctrl_on(uint8_t num)
{
    if (num > POWER4_CTRL_SWITCH)
    {
        return;
    }
    GPIO_SetBits(GPIOH, GPIO_Pin_2 << num);
}

void power_ctrl_off(uint8_t num)
{
    if (num > POWER4_CTRL_SWITCH)
    {
        return;
    }
    GPIO_ResetBits(GPIOH, GPIO_Pin_2 << num);
}

void power_ctrl_toggle(uint8_t num)
{
    if (num > POWER4_CTRL_SWITCH)
    {
        return;
    }
    GPIO_ToggleBits(GPIOH, GPIO_Pin_2 << num);
}

void power_init(void){
    //24输出控制口 初始化
    power_ctrl_configuration();

    //24v 输出 依次上电
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++){
        power_ctrl_on(i);
        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }
}
