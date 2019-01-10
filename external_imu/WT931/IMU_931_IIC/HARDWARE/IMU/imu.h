#ifndef __IMU_H__
#define __IMU_H__
#include "myiic.h"	

#define ROLL_REG				0x3d
#define PITCH_REG				0x3e
#define YAW_REG			  	0x3f

void get_yaw_picth_roll_angle(float *pitch,float *roll,float *yaw);

#endif
