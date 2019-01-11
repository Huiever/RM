#ifndef _ANGLE_H
#define _ANGLE_H
#include "mpu9250.h"
#include "timer.h"
#include "math.h"

void Init_Quaternion(void);
void GetPitchYawGxGyGz(void);
void IMU_AHRSupdate(void);
void IMU_getValues(void);
extern volatile float yaw,pitch,roll; //使用到的角度值，这里是最终的三轴数据

#endif
