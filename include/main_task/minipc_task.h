#ifndef _MINIPC_TASK_H_
#define _MINIPC_TASK_H_
#include "sys.h"

typedef struct {
    uint8_t start1;
    uint8_t start2;
    uint8_t cmdid;
    uint8_t data[4]; 
    uint8_t end1;
    uint8_t end2;
}package_t;

typedef enum {
    GIMBAL_CMD_STOP,
    GIMBAL_CMD_MOVEBY,
    GIMBAL_CMD_MOVETO
}GimbalMovingCtrType_e;

typedef struct{
    uint8_t startFriction;
    GimbalMovingCtrType_e gimbalMovingCtrType;
    float d1;
    float d2;
}UpperMonitor_Ctr_t;

UpperMonitor_Ctr_t GetUpperMonitorCmd(void);
uint8_t GetUpperMonitorOnline(void);
void SetUpperMonitorOnline(uint8_t isOnline);
void UpperMonitorDataProcess(volatile uint8_t *pData);

void RequestFinishFrictionSpeedUp(void);
void miniPC_task(void);

#endif
