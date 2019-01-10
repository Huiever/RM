#include "main.h"

int main(void){ 
	float pitch,roll,yaw; 		//Å·À­½Ç
  DRIVER_INIT();	
	IIC_Init();
	while (1)
	{
		get_yaw_picth_roll_angle(&pitch,&roll,&yaw);
		printf("%f,%f,%f\r\n",pitch,roll,yaw);
  }
}
