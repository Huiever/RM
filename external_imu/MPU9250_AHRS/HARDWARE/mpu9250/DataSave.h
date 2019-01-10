#ifndef _DATASAVE_H
#define _DATASAVE_H

#include "mpu9250.h"

extern IMUDataTypedef IMU_Real_Data;
extern IMUDataTypedef IMU_RAW_Data;
void AK8963_mgetValues(volatile float *arry);
void MPU6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
void AK8963_getlastValues(int16_t *x,int16_t *y,int16_t *z);

void Recorret_offset_g(void);

#endif
