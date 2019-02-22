#include "main.h"
#include "bsp.h"
#include "imu.h"
#include "beep.h" 
#include "usart3.h"
#include "delay.h"
#include "can_bus_task.h"
#include "gun.h"
#include "visualscope.h"
#include "control_task.h"

#include "myiic.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 

int main(void){
    BSP_Pre_Init();
    delay_ms(5000);
    if(Check_Sum > 1){
        printf("Check your monitors!\r\n");
        Sing_bad_case();
    }
    //float pitch,roll,yaw; 		//ŷ����
    BSP_Init();
//	short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
//	short gyrox,gyroy,gyroz;	//������ԭʼ����
//	short temp;				     	//�¶� 
//	while(mpu_dmp_init());
// 	while(1)
//	{
//		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
//		{ 
//			temp=MPU_Get_Temperature();	//�õ��¶�ֵ
//			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
//			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������			
//		}
//			//printf("pitch:%f roll:%f yaw:%f\r\n",pitch,roll,yaw);
//		printf("aacx:%d\r\n",aacx);
//		//VisualScope(USART1,pitch ,roll,yaw,0);
//	} 	
    while(1){
       // printf("%8f, %8.3lf, %8.3lf\r\n", imu.rip.yaw, imu.rip.pit,imu.rip.rol);
//      printf("%8f\r\n", imu.rip.temp);
//      float x=0;
//      x=imu.rip.yaw-imu.rip.pit;
//      VisualScope(USART3, imu.rip.yaw, imu.rip.pit, x, 0);
        imu_main();
    }
}
