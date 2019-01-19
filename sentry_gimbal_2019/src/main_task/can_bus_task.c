#include "can_bus_task.h"
#include "stdio.h"
#include "can1.h"


int16_t  pitch_ecd_bias_offset = 3000;
int16_t  yaw_ecd_bias_offset   = 1160;

static uint32_t can_count = 0;

volatile Encoder GMYawEncoder   = {0,0,0,0,0,0,0,0,0};
volatile Encoder GMPitchEncoder = {0,0,0,0,0,0,0,0,0};

rammer Rammer = { 0, 0, 0 };

uint16_t Infantry_HeatData    = 0;
uint8_t  Flag_In_RunAwayState = 0;

void EncoderProcess(volatile Encoder *v, CanRxMsg * msg){
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
		v->ecd_raw_rate = v->diff- 8192;
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
			if(can_count==1){
				GMYawEncoder.ecd_bias = yaw_ecd_bias_offset;
				GMPitchEncoder.ecd_bias = pitch_ecd_bias_offset;
			}
			EncoderProcess(&GMYawEncoder ,msg);
			if(can_count>=90 && can_count<=100){
				if((GMYawEncoder.ecd_bias - GMYawEncoder.ecd_value) <-4000){
					GMYawEncoder.ecd_bias = yaw_ecd_bias_offset + 8192;
				}
				else if((GMYawEncoder.ecd_bias - GMYawEncoder.ecd_value) > 4000){
					GMYawEncoder.ecd_bias = yaw_ecd_bias_offset - 8192;
				}
			}
		}break;
		case CAN_BUS1_Pitch_FEEDBACK_MSG_ID:{
			if(can_count==1){
				GMYawEncoder.ecd_bias = yaw_ecd_bias_offset;
				GMPitchEncoder.ecd_bias = pitch_ecd_bias_offset;
			}
			EncoderProcess(&GMPitchEncoder ,msg);
			if(can_count>=90 && can_count<=100){
				if((GMPitchEncoder.ecd_bias - GMPitchEncoder.ecd_value) <-4000){
					GMPitchEncoder.ecd_bias = pitch_ecd_bias_offset + 8192;
				}
				else if((GMPitchEncoder.ecd_bias - GMPitchEncoder.ecd_value) > 4000){
					GMPitchEncoder.ecd_bias = pitch_ecd_bias_offset - 8192;
				}
			}
		}break;
		case CAN_BUS1_Rammer_FEEDBACK_MSG_ID:{
			Rammer.angle  = msg->Data[0] << 8 | msg->Data[1];
			Rammer.speed  = msg->Data[2] << 8 | msg->Data[3];
			Rammer.torque = msg->Data[4] << 8 | msg->Data[5];
		}break;
		case ChassisSensor_ID:{	
			Infantry_HeatData    = msg->Data[0]<<8 | msg->Data[1];
			Flag_In_RunAwayState = msg->Data[2];
		}
		default:{
			
		}break;
	}
}

void Set_Gimbal_Current(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq){
    CanTxMsg tx_message;
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = 0x00;
    tx_message.Data[1] = 0x00;
    tx_message.Data[2] = (unsigned char)(gimbal_yaw_iq >> 8);
    tx_message.Data[3] = (unsigned char)gimbal_yaw_iq;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
    tx_message.StdId = 0x200;
    tx_message.Data[0] = 0x00;
    tx_message.Data[1] = 0x00;
    tx_message.Data[2] = 0x00;
    tx_message.Data[3] = 0x00;
    tx_message.Data[4] = (unsigned char)(gimbal_pitch_iq >> 8);
    tx_message.Data[5] = (unsigned char)gimbal_pitch_iq;
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
    tx_message.Data[0] = 0x00;
    tx_message.Data[1] = 0x00;
    tx_message.Data[2] = 0x00;
    tx_message.Data[3] = 0x00;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = (uint8_t)(rammer_current >> 8);
    tx_message.Data[7] = (uint8_t)rammer_current;
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

uint16_t Get_Infantry_HeatData(void){
	return Infantry_HeatData;
}

uint8_t  Get_Flag_In_RunAwayState(void){
	return Flag_In_RunAwayState;
}
