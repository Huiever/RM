#include <stm32f4xx.h>
#include "remote_task.h"
#include "ramp.h"
#include "gun.h"
#include "usart6.h"
#include "control_task.h"
#include "stdio.h"

static ControlMode_e controlmode = STOP;

RC_Ctl_t           RC_CtrlData;
Gimbal_Ref_t       GimbalRef;
Gimbal_Target_t    Gimbal_Target;
RampGen_t          FrictionRamp = RAMP_GEN_DAFAULT;

static  uint8_t Flag_AutoShoot     = 0;
int16_t t_inversion        = 0;
uint8_t Frion_Flag         = 0;
int16_t MiniPC_Alive_Count = 0;

int16_t ChassisSpeed_Target = 0;

FrictionWheelState_e friction_wheel_state = FRICTION_WHEEL_OFF;
int FRICTION_WHEEL_MAX_DUTY = 1250;

volatile uint8_t upperMonitorOnline = 0;
static   UpperMonitor_Ctr_t upperMonitorCmd = {0,GIMBAL_CMD_STOP,0,0};

void UpperMonitorDataProcess(uint8_t *pData){
    static const uint8_t START_UPPER_MONITOR_CTR = 0x00;
    static const uint8_t GIMBAL_MOVETO           = 0x01;
    static const uint8_t GIMBAL_MOVEBY           = 0x02;
    static const uint8_t GIMBAL_TURN_BACK        = 0x03;
    static const uint8_t START_FRICTION          = 0x04;
    static const uint8_t STOP_FRICTION           = 0x05;
    static const uint8_t START_SHOOTING          = 0x06;
    static const uint8_t STOP_SHOOTING           = 0x07;
    static const uint8_t REQUEST_CURR_STATE      = 0x08;
    static const uint8_t EXIT_UPPER_MONITOR_CTR  = 0x09;
    static const uint8_t MINIPC_ALIVE            = 0x0F;
    int16_t d1 = *((int16_t *)(pData + 1));
    int16_t d2 = *((int16_t *)(pData + 3));
    if(upperMonitorOnline){
        switch (pData[0]){
            case GIMBAL_MOVETO:{
                upperMonitorCmd.d1 = d1 * 0.01f;
                upperMonitorCmd.d2 = d2 * 0.01f;
                upperMonitorCmd.gimbalMovingCtrType = GIMBAL_CMD_MOVETO;
            }break;
            
            case GIMBAL_MOVEBY:{
                upperMonitorCmd.d1 = d1 * 0.001f;
                upperMonitorCmd.d2 = d2 * 0.001f;
                upperMonitorCmd.gimbalMovingCtrType = GIMBAL_CMD_MOVEBY;
            }break;
            
            case GIMBAL_TURN_BACK:{
                upperMonitorCmd.gimbalMovingCtrType = GIMBAL_CMD_STOP;
            }break;
            
            case START_FRICTION:{
                upperMonitorCmd.startFriction = 1;
                SetFrictionWheelSpeed(1000 + (FRICTION_WHEEL_MAX_DUTY-1000)*FrictionRamp.Calc(&FrictionRamp)); 
                if(FrictionRamp.IsOverflow(&FrictionRamp)){
                    RequestFinishFrictionSpeedUp();
                }
            }break;
                
            case STOP_FRICTION:{
                upperMonitorCmd.startFriction = 0;
                SetFrictionWheelSpeed(1000);
                FrictionRamp.ResetCounter(&FrictionRamp);
            }break;
            
            case START_SHOOTING:{
                Set_Flag_AutoShoot(1);
            }break;
            
            case STOP_SHOOTING:{
                Set_Flag_AutoShoot(0);
            }break;
            
            case REQUEST_CURR_STATE:{
                
            }break;
            
            case EXIT_UPPER_MONITOR_CTR:{
                SetUpperMonitorOnline(0);
            }break;

            case MINIPC_ALIVE:{
                Reset_MiniPC_Alive_Count();
            }break;
            
            default:{
                
            }break;
        }
    }
    else if(pData[0] == START_UPPER_MONITOR_CTR){
                SetUpperMonitorOnline(1);
        }
    else if(pData[0] == MINIPC_ALIVE){
    Reset_MiniPC_Alive_Count();
}
}

uint8_t Get_Flag_AutoShoot(void){
    return Flag_AutoShoot;
}

void Set_Flag_AutoShoot(uint8_t flag){
    Flag_AutoShoot = flag;
}

void Reset_MiniPC_Alive_Count(void){
    MiniPC_Alive_Count = 0;
}

int16_t Get_MiniPC_Alive_Count(void){
    return MiniPC_Alive_Count;
}

void MiniPC_Alive_Count_PlusPlus(void){
    MiniPC_Alive_Count++;
}

void Set_FRICTION_WHEEL_MAX_DUTY( uint32_t x ){
    FRICTION_WHEEL_MAX_DUTY = x;
}

void Set_t_inversion(int16_t t){
    t_inversion = t;
}

int16_t Get_t_inversion(void){
    return t_inversion;
}

UpperMonitor_Ctr_t GetUpperMonitorCmd(void){
    return upperMonitorCmd;
}

FrictionWheelState_e GetFrictionState(){
    return friction_wheel_state;
}

void SetFrictionState(FrictionWheelState_e v){
    friction_wheel_state = v;
}

void SetUpperMonitorOnline(uint8_t isOnline){
    upperMonitorOnline = isOnline;
}

uint8_t GetUpperMonitorOnline(void){
    return upperMonitorOnline;
}

void ResetUpperMonitorCmd(void){
    upperMonitorCmd.d1 = 0;
    upperMonitorCmd.d2 = 0;
    SetFrictionWheelSpeed(1000);
    FrictionRamp.ResetCounter(&FrictionRamp);
    Set_Flag_AutoShoot(0);
    SetUpperMonitorOnline(0);
    SetWorkState(CRUISE_STATE);
}

void ResetUpperMonitorCmd_d1_d2(void){
    upperMonitorCmd.d1 = 0;
    upperMonitorCmd.d2 = 0;
}

void RequestFinishFrictionSpeedUp(void){
    UART6_PrintCh('1');
}

void GimbalAngleLimit(void){
    VAL_LIMIT(Gimbal_Target.pitch_angle_target, PITCH_MIN + 5.0f, PITCH_MAX-5.0f);
//    VAL_LIMIT(Gimbal_Target.yaw_angle_target, 177.0f-YAW_MAX, 177.0f+YAW_MAX);
}

void RemoteDataProcess(uint8_t *pData){
    switch(GetWorkState()){
        case PREPARE_STATE:{

        }break;
        default:{
            if(pData == NULL){
                    return;
            }
            RC_CtrlData.rc.ch0  = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
            RC_CtrlData.rc.ch1  = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
            RC_CtrlData.rc.ch2  = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
                                  ((int16_t)pData[4] << 10)) & 0x07FF;
            RC_CtrlData.rc.ch3  = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
            RC_CtrlData.rc.s1   = ((pData[5] >> 4) & 0x000C) >> 2;
            RC_CtrlData.rc.s2   = ((pData[5] >> 4) & 0x0003);
            RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
            RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
            RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);
            RC_CtrlData.mouse.press_l = pData[12];
            RC_CtrlData.mouse.press_r = pData[13];
            RC_CtrlData.key.v   = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);
            SetControlMode(&RC_CtrlData.rc);
            if(GetWorkState() == CONTROL_STATE){
                ChassisSpeed_Target = (RC_CtrlData.rc.ch1 - 1024) * 10;
                
                Gimbal_Target.pitch_angle_target += (float)(RC_CtrlData.rc.ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET)
                                                    * STICK_TO_PITCH_ANGLE_INC_FACT;
//                if(Get_Flag(CM360rotate)==0)
//                {
//                Gimbal_Target.yaw_angle_target   += (RC_CtrlData.rc.ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET)
//                                                    * STICK_TO_YAW_ANGLE_INC_FACT;
//                }
                
                printf("%6.3f\r\n",Gimbal_Target.pitch_angle_target);

                if( RC_CtrlData.rc.s1 == 1 ){
                    SetFrictionWheelSpeed(1000);
                    Set_Flag_AutoShoot(0);
                }
                else if( RC_CtrlData.rc.s1 == 3 ){
                    SetFrictionWheelSpeed(1250);
                    Set_Flag_AutoShoot(0);
                }
                else{
                    SetFrictionWheelSpeed(1250);
                    Set_Flag_AutoShoot(1);
                }
            }
            GimbalAngleLimit();
        }
    }
}

void SetControlMode(Remote *rc){
    if(rc->s2 == 1){
        controlmode = REMOTE_CONTROL;
    }
    else if(rc->s2 == 3){
        controlmode = AUTO_CONTROL;
    }
    else if(rc->s2 == 2){
        controlmode = STOP;
    }
}

ControlMode_e GetControlMode(void){
    return controlmode;
}

uint8_t Is_Control_State(void){
    if(GetControlMode() == REMOTE_CONTROL){
        return 1;
    }
    else{
        return 0;
    }
}

int16_t Get_ChassisSpeed_Target(void){
    return ChassisSpeed_Target;
}

void Reset_ChassisSpeed_Target(void){
    ChassisSpeed_Target = 0;
}

void RemoteTaskInit(void){
  FrictionRamp.SetScale(&FrictionRamp, FRICTION_RAMP_TICK_COUNT);
  FrictionRamp.ResetCounter(&FrictionRamp);
  Gimbal_Target.pitch_angle_target = 0.0f;
  Gimbal_Target.yaw_angle_target   = 0.0f;
  SetFrictionState(FRICTION_WHEEL_OFF);
}
