#ifndef _CAN_BUS_TASK_H_
#define _CAN_BUS_TASK_H_

#include "can1.h"

#define ChassisSensor_ID                        0x402
#define CAN_BUS1_Yaw_FEEDBACK_MSG_ID            0x205
#define CAN_BUS1_Pitch_FEEDBACK_MSG_ID          0x206
#define CAN_BUS1_Rammer_FEEDBACK_MSG_ID         0x201
#define CAN_BUS1_Rammer_SEND_MSG_ID             0x200

#define RATE_BUF_SIZE 6

typedef struct{
    int32_t raw_value;
    int32_t last_raw_value;
    int32_t ecd_value;
    int32_t diff;
    int32_t temp_count;
    uint8_t buf_count;
    int32_t ecd_bias;
    int32_t ecd_raw_rate;
    int32_t rate_buf[RATE_BUF_SIZE];
    int32_t round_cnt;
    int32_t filter_rate;
    float ecd_angle;
}Encoder;

typedef struct{
    int16_t angle;
    int16_t speed;
    int16_t torque;
}rammer;

extern volatile rammer Rammer;
extern volatile Encoder GMYawEncoder;
extern volatile Encoder GMPitchEncoder;

uint16_t Get_Sentry_HeatData(void);
uint8_t  Get_Flag_In_RunAwayState(void);
void CanReceiveMsgProcess(CanRxMsg *msg);
void EncoderProcess(volatile Encoder *v, CanRxMsg *msg);
void Send_Gimbal_Info(uint8_t Flag_Shoot_State, uint8_t Control_Mode, int16_t ChassisSpeed);
void Set_Gimbal_Current(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq);
void Set_Rammer_Current(CAN_TypeDef *CANx, int16_t rammer_current);

#endif

