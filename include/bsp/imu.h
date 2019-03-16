#ifndef _IMU_H_
#define _IMU_H_

#include "sys.h"

#define MPU6500_NSS_Low() GPIO_WriteBit(GPIOF, GPIO_Pin_6, Bit_RESET)
#define MPU6500_NSS_High() GPIO_WriteBit(GPIOF, GPIO_Pin_6, Bit_SET)

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500控制温度的设置TIM的重载值，即给PWM最大为 MPU6500_TEMP_PWM_MAX - 1
#define GYRO_CONST_MAX_TEMP 45.0f //陀螺仪控制恒温 最大控制温度

#define MPU6500_TEMPERATURE_FACTOR 0.002f
#define MPU6500_TEMPERATURE_OFFSET 23.0f

#define MPU6500_TEMPERATURE_PID_KP 1600.0f //温度控制PID的kp
#define MPU6500_TEMPERATURE_PID_KI 0.2f    //温度控制PID的ki
#define MPU6500_TEMPERATURE_PID_KD 0.0f    //温度控制PID的kd

#define MPU6500_TEMPERATURE_PID_MAX_OUT 4500.0f  //温度控制PID的max_out
#define MPU6500_TEMPERATURE_PID_MAX_IOUT 4400.0f //温度控制PID的max_iout

#define IMU_Temperature_PID_DEFAULT \
{\
    0,\
    0,\
    {0,0},\
    MPU6500_TEMPERATURE_PID_KP,\
    MPU6500_TEMPERATURE_PID_KI,\
    MPU6500_TEMPERATURE_PID_KD,\
    0,\
    0,\
    0,\
    0,\
    0,\
    0,\
    0,\
    MPU6500_TEMPERATURE_PID_MAX_OUT,\
    0,\
    0,\
    0,\
    &PID_Calc,\
    &PID_Reset,\
}

typedef struct{
        float ax; //unit: m/s2
        float ay;
        float az;
        
        float gx; /*!< omiga, +- 1000dps -> rad/s */
        float gy;
        float gz;
        
        float mx; //unit: G
        float my;
        float mz;

        float temp;

        float rol;
        float pit;
        float yaw;
}ripdata_t;  //处理过的数据

typedef struct{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;
    
    struct{
        //修正磁强畸变
        float mx;    //unit: mG
        float my;
        float mz;
      //修正椭球畸变
        float b0;
        float b1;
        float b2;
        float b3;
        float b4;
        float b5;
    }mag;
}offset_t;  //偏移

typedef struct{
        int16_t ax;
        int16_t ay;
        int16_t az;
     
        int16_t temp;

        int16_t gx;
        int16_t gy;
        int16_t gz;
        
        int16_t mx;            //unit: mG
        int16_t my;
        int16_t mz;
}rawdata_t;     //原始数据

typedef struct{
    rawdata_t raw;
    offset_t offset;
    ripdata_t rip;
} imu_t;

void imu_init(void);
void imu_main(void);
void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z);

float get_yaw_angle(void);        //获得yaw角度
float get_pit_angle(void);        //获得pit角度

float get_imu_wx(void);
float get_imu_wy(void);
float get_imu_wz(void);

//----------------------------------------------------------------------------------------------------
// Variable declaration
extern imu_t imu;
extern volatile float twoKp;             // 2 * proportional gain (Kp)
extern volatile float twoKi;             // 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3;    // quaternion of sensor frame relative to auxiliary frame

extern uint8_t MPU6500_Write_Regss(uint8_t const addr,uint8_t reg,uint8_t len,uint8_t *buf);
extern uint8_t MPU6500_Read_Regss(uint8_t const addr,uint8_t reg,uint8_t len,uint8_t *buf);

#endif
