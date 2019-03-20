#include "can_bus_task.h"
#include "stdio.h"
#include "can1.h"
#include "main.h"

static uint32_t can_count = 0;

volatile Encoder GMYawEncoder   = {0,0,0,0,0,0,0,0,0};
volatile Encoder GMPitchEncoder = {0,0,0,0,0,0,Pit_Encoder_Offset,0,0};

volatile rammer Rammer = { 0, 0, 0 };

volatile uint16_t Sentry_HeatData    = 0;
volatile uint8_t  Flag_In_RunAwayState = 0;

void EncoderProcess(volatile Encoder *v, CanRxMsg *msg){
    int i = 0;
    int32_t temp_sum  = 0;    
    v->last_raw_value = v->raw_value;
    v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
    v->diff = v->raw_value - v->last_raw_value;
    if(v->diff < -7500){
        v->round_cnt++;
        v->ecd_raw_rate = v->diff + 8192;
    }
    else if(v->diff > 7500){
        v->round_cnt--;
        v->ecd_raw_rate = v->diff - 8192;
    }        
    else{
        v->ecd_raw_rate = v->diff;
    }
    v->ecd_value = v->raw_value + v->round_cnt * 8192;
    v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
    v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
    if(v->buf_count == RATE_BUF_SIZE){
        v->buf_count = 0;
    }
    for(i = 0;i < RATE_BUF_SIZE; i++){
        temp_sum += v->rate_buf[i];
    }
    v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);
}

void CanReceiveMsgProcess(CanRxMsg * msg){
    can_count++;
    switch(msg->StdId){
        case CAN_BUS1_Yaw_FEEDBACK_MSG_ID:{
            EncoderProcess(&GMYawEncoder, msg);
        }break;
        case CAN_BUS1_Pitch_FEEDBACK_MSG_ID:{
            EncoderProcess(&GMPitchEncoder, msg);
//            if(can_count>=90 && can_count<=100){
//                if((GMPitchEncoder.ecd_bias - GMPitchEncoder.ecd_value) <-4000){
//                    GMPitchEncoder.ecd_bias = Pit_Encoder_Offset + 8192;
//                }
//                else if((GMPitchEncoder.ecd_bias - GMPitchEncoder.ecd_value) > 4000){
//                    GMPitchEncoder.ecd_bias = Pit_Encoder_Offset - 8192;
//                }
//            }
        }break;
        case CAN_BUS1_Rammer_FEEDBACK_MSG_ID:{
            Rammer.angle  = msg->Data[0] << 8 | msg->Data[1];
            Rammer.speed  = msg->Data[2] << 8 | msg->Data[3];
            Rammer.torque = msg->Data[4] << 8 | msg->Data[5];
        }break;
        case ChassisSensor_ID:{
            Sentry_HeatData    = msg->Data[0]<<8 | msg->Data[1];
            Flag_In_RunAwayState = msg->Data[2];
        }
        default:{
            
        }break;
    }

#if Monitor_GM_Encoder==1
    printf("yaw_raw_value:%6.6d  pit_raw_value:%6.6d  yaw_ecd:%6.6f  pit_ecd:%6.6f\r\n",
            GMYawEncoder.raw_value, GMPitchEncoder.raw_value,GMYawEncoder.ecd_angle, GMPitchEncoder.ecd_angle);
#endif

#if Monitor_rammer==1
    printf("angle:%6d  speed:%6d  torque:%6d\r\n",
            Rammer.angle, Rammer.speed, Rammer.torque);
#endif
}

void Set_Gimbal_Current(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq){
    CanTxMsg tx_message;
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (unsigned char)(gimbal_yaw_iq >> 8);
    tx_message.Data[1] = (unsigned char)gimbal_yaw_iq;
    tx_message.Data[2] = (unsigned char)(gimbal_pitch_iq >> 8);
    tx_message.Data[3] = (unsigned char)gimbal_pitch_iq;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
}

void Set_Rammer_Current(CAN_TypeDef *CANx, int16_t rammer_current ){
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (uint8_t)(rammer_current >> 8);
    tx_message.Data[1] = (uint8_t)rammer_current;
    tx_message.Data[2] = 0x00;
    tx_message.Data[3] = 0x00;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
}

void Send_Gimbal_Info(uint8_t Flag_Shoot_State, uint8_t Control_Mode, int16_t ChassisSpeed){
    CanTxMsg tx_message;
    tx_message.StdId = 0x401;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (uint8_t)Flag_Shoot_State;
    tx_message.Data[1] = (uint8_t)Control_Mode;
    tx_message.Data[2] = (uint8_t)(ChassisSpeed >> 8);
    tx_message.Data[3] = (uint8_t)ChassisSpeed;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CAN1,&tx_message);
}

uint16_t Get_Sentry_HeatData(void){
    return Sentry_HeatData;
}

uint8_t  Get_Flag_In_RunAwayState(void){
    return Flag_In_RunAwayState;
}
