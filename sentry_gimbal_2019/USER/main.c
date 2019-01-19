#include "bsp.h"
#include "imu.h"
#include "beep.h"
#include "usart3.h"
#include "delay.h"
#include "can_bus_task.h"
#include "main.h"

int main(void){
	BSP_Pre_Init();
	delay_ms(5000);
	if(Check_Sum > 1){
	  printf("Check your monitors!\r\n");
		Sing_bad_case();
	}
 
	BSP_Init();
	while(1){ 
		//Set_Gimbal_Current(CAN1, 50, 50);	
		imu_main();
	}
}
