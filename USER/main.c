#include "bsp.h"
#include "imu.h"
#include "beep.h" 
#include "usart3.h"
#include "delay.h"
#include "can_bus_task.h"
#include "main.h"
#include "gun.h"
#include "visualscope.h"
#include "control_task.h"
int main(void){
	BSP_Pre_Init();
	delay_ms(5000);
	if(Check_Sum > 1){
	  printf("Check your monitors!\r\n");
		Sing_bad_case();
	}
	
	BSP_Init();
	while(1){ 
		//VisualScope(USART3,RAMMERSpeedPID.output,Rammer.speed,TargetSpeed,0);
		imu_main();
	}
}
