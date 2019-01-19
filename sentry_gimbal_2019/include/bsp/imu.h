#ifndef _IMU_H_
#define _IMU_H_

#include "sys.h"

#define MPU6500_NSS_Low() GPIO_WriteBit(GPIOF, GPIO_Pin_6, Bit_RESET)
#define MPU6500_NSS_High() GPIO_WriteBit(GPIOF, GPIO_Pin_6, Bit_SET)

typedef struct
{
	struct
	{
		int16_t ax;     
		int16_t ay;
		int16_t az;
	 
		int16_t temp;

		int16_t gx;     
		int16_t gy;
		int16_t gz;
		
		int16_t mx;			//unit: mG
		int16_t my;
		int16_t mz;
	}raw;	 //ԭʼ����
	struct
	{
		int16_t ax;     
		int16_t ay;
		int16_t az;
	 
		int16_t gx;     
		int16_t gy;
		int16_t gz;
		
		struct{
			//������ǿ����
			float mx;    //unit: mG
			float my;
			float mz;
		  //�����������	
			float b0;
			float b1;
			float b2;
			float b3;
			float b4;
			float b5;
		}mag;
		
	}offset;  //ƫ��	
	struct
	{
		float ax; //unit: g
		float ay;
		float az;
		
		float gx; /*!< omiga, +- 2000dps => +-32768  so gx/16.4/57.3f =	rad/s */
		float gy;
		float gz;
		
		float mx; //unit: G
		float my;
		float mz;

		float temp;

		float rol;
		float pit;
		float yaw;
	}rip;  //�����������
} imu_t;

uint8_t MPU6500_Init(void);
void IMU_Get_Raw_Data(void);
void mpu_offset_call(void);
void init_quaternion(void);
void IMU_AHRSupdate(void);
void IMU_getYawPitchRoll(void);
float get_yaw_angle(void);
float get_imu_wx(void);
float get_imu_wy(void);
float get_imu_wz(void);
int16_t get_mpu_gx(void);
int16_t get_mpu_gy(void);
int16_t get_mpu_gz(void);

void imu_main(void);

#endif
