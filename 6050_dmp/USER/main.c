#include "main.h"

int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);  //��ʼ����ʱ����
	uart_init(115200);		//��ʼ�����ڲ�����Ϊ115200
	//MPU_Init();					//��ʼ��MPU6050
 	float pitch,roll,yaw; 		//ŷ����
//	short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
//	short gyrox,gyroy,gyroz;	//������ԭʼ����
//	short temp;				     	//�¶� 
	while(mpu_dmp_init());
 	while(1)
	{
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{ 
//			temp=MPU_Get_Temperature();	//�õ��¶�ֵ
//			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
//			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������			
		}
			printf("pitch:%f roll:%f yaw:%f\r\n",pitch,roll,yaw);
		//VisualScope(USART1,pitch ,roll,yaw,0);
	} 	
}
