//#ifndef __MINIPC_H_
//#define __MINIPC_H_

//#include "sys.h"
////单发打完
////
//typedef enum {
//    START_UPPER_MONITOR_CTR = 0x00,
//    REQUEST_PITCH = 0x01,
//    GIMBAL_MOVEBY = 0x02,
//    GIMBAL_STOP = 0x03,
//    START_FRICTION = 0x04,
//    STOP_FRICTION = 0x05,
//    START_SHOOTING = 0x06,
//    STOP_SHOOTING = 0x07,
//    REQUEST_CURR_STATE = 0x08,
//    SYN = 0x09,
//    ACK = 0x10,
//    ACK_STATUS = 0x11,
//    EXIT_UPPER_MONITOR_CTR = 0x12,
//    NO_CMD = 0xff
//}minipc_cmdid_e;

//typedef struct {
//    uint8_t start1;
//    uint8_t start2;
//    uint8_t cmdid;
//    uint8_t data[4];
//    uint8_t end1;
//    uint8_t end2;
//}package_t;

//typedef struct {
//    uint8_t startFriction;
//    uint8_t cmdid;
//    float d1;
//    float d2;
//}UpperMonitor_Ctr_t;

//extern UpperMonitor_Ctr_t upperMonitorCmd;

//void UpperMonitorDataProcess(uint8_t *pData);
//UpperMonitor_Ctr_t GetUpperMonitorCmd(void);
//void miniPC_BS_Start(int8_t pitch_angle);
//void miniPC_BS_end(int8_t pitch_angle);
//void miniPC_BS_normal(int8_t pitch_angle);
//void miniPC_HS_ACK(void);
//void miniPC_HS_SYN(void);
//void miniPC_ACK_status(void);
//void miniPC_task(void);
//void miniPC_Init(void);
//uint8_t GetUpperMonitorOnline(void);

//#endif
