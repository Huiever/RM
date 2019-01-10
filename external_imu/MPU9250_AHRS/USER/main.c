#include "main.h"


int main(void){ 
	
  DRIVER_INIT();
  MPU9250_Init();
	
	while(1){
		IMU_getYawPitchRoll(angle);
		GetPitchYawGxGyGz();
	  printf("yaw:%f pitch:%f roll:%f\r\n",yaw_angle,pitch_angle,roll_angle);
	}
}
