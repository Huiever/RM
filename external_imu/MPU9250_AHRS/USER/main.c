#include "main.h"


int main(void){ 
	
  DRIVER_INIT();
  MPU9250_Init();
	
	while(1){
		
		IMU_getValues();	 //获取原始数据,加速度计和磁力计是原始值，陀螺仪转换成了deg/s
    IMU_AHRSupdate();
		//filterUpdate(imu.rip.gx,imu.rip.gy,imu.rip.gz,imu.raw.ax,imu.raw.ay,imu.raw.az);
		GetPitchYawGxGyGz();

	  printf("yaw:%f pit:%f rol:%f\r\n",imu.rip.yaw,imu.rip.pit,imu.rip.rol);
//		printf("mx:%d my:%d mz:%d ax:%d ay:%d az:%d gx:%d gy:%d gz:%d\r\n"
//						,imu.raw.mx,imu.raw.my,imu.raw.mz
//						,imu.raw.ax,imu.raw.ay,imu.raw.az
//						,imu.raw.gx,imu.raw.gy,imu.raw.gz);
		
	}
}
