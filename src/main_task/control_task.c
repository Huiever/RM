#include "control_task.h"
#include "pid_regulator.h"
#include "ramp.h"
#include "remote_task.h"
#include "flags.h"
#include "can_bus_task.h"
#include "imu.h"
#include "sys.h"
#include "remote_task.h"

PID_Regulator_t GMPPositionPID = GIMBAL_MOTOR_PITCH_POSITION_PID_DEFAULT;
PID_Regulator_t GMPSpeedPID    = GIMBAL_MOTOR_PITCH_SPEED_PID_DEFAULT;
PID_Regulator_t GMYPositionPID = GIMBAL_MOTOR_YAW_POSITION_PID_DEFAULT;
PID_Regulator_t GMYSpeedPID    = GIMBAL_MOTOR_YAW_SPEED_PID_DEFAULT;
PID_Regulator_t RAMMERSpeedPID = RAMMER_SPEED_PID_DEFAULT;

WorkState_e lastWorkState = PREPARE_STATE;
WorkState_e workState     = PREPARE_STATE;

int8_t   shooting    = 0;
uint16_t HeatLimit   = 360;
uint8_t  ShootSpeed  = 25;

RampGen_t GMPitchRamp = RAMP_GEN_DAFAULT;
RampGen_t GMYawRamp   = RAMP_GEN_DAFAULT;

static uint32_t time_tick_1ms = 0;

void Control_Task(void){
    time_tick_1ms++;
    WorkStateFSM();
    MiniPC_Alive_Count_PlusPlus();
    if(Get_MiniPC_Alive_Count() > 3000 && GetWorkState() == SHOOT_STATE){
        ResetUpperMonitorCmd();
    }
    if(GetWorkState() == SHOOT_STATE){
        UpperMonitorControlLoop();
    }
    GimbalYawControlModeSwitch();
    GMPitchControlLoop();
    GMYawControlLoop();
#if Monitor_GM_Encoder==0
    SetGimbalMotorOutput();
#endif
    if(time_tick_1ms % 2 == 0){
        ShootControlLoop();
    }
    else{
        //Send_Gimbal_Info(GetUpperMonitorOnline(),Is_Control_State(),Get_ChassisSpeed_Target());
    }
}

void WorkStateFSM(void){
    lastWorkState = workState;
    switch(workState){
        case PREPARE_STATE:{
            if(time_tick_1ms > PREPARE_TIME_TICK_MS)
                workState = CRUISE_STATE;
        }break;
        case CRUISE_STATE:{
            if(GetUpperMonitorOnline() == 1){
                workState = SHOOT_STATE;
            }
            if(Get_Flag_In_RunAwayState() == 1){
                workState = RUNAWAY_STATE;
            }
            if(GetControlMode() == REMOTE_CONTROL){
                workState = CONTROL_STATE;
            }
        }break;
        case SHOOT_STATE:{
            if(GetUpperMonitorOnline() == 0){
                workState = CRUISE_STATE;
            }
            if(Get_Flag_In_RunAwayState() == 1){
                workState = RUNAWAY_STATE;
            }
            Send_Gimbal_Info(1,0,0);
        }break;
        case RUNAWAY_STATE:{
            if(Get_Flag_In_RunAwayState() == 0){
                workState = CRUISE_STATE;
            }
        }break;
        case STOP_STATE:{
            
        }break;
        case CONTROL_STATE:{
            if(GetControlMode() != REMOTE_CONTROL){
                workState = CRUISE_STATE;
                Reset_ChassisSpeed_Target();
            }
            if(GetUpperMonitorOnline() == 1){
                workState = SHOOT_STATE;
                Reset_ChassisSpeed_Target();
            }
            Send_Gimbal_Info(0,1,Get_ChassisSpeed_Target());
        }break;
    }
}

void SetWorkState(WorkState_e state){
    workState = state;
}

WorkState_e GetWorkState(void){
    return workState;
}

void GimbalYawControlModeSwitch(void){
    static uint8_t  CruiseFlag    = 0;
    static uint8_t  ShootFlag     = 0;
    static uint8_t  RunAwayFlag   = 0;
    static uint8_t  ControlFlag   = 0;
    static float    YawAngleSave  = 0.0f;
    static uint8_t  Flag_Cruise_Reverse = 0;
    static int16_t  Cruise_Time_Between = 0;
    switch(GetWorkState()){
        case PREPARE_STATE:{
            GMYPositionPID.ref = 177.0f;
            GMYPositionPID.fdb = 177.0f;
            YawAngleSave = GMYawEncoder.ecd_angle;
        }break;
        case CRUISE_STATE:{
            if(CruiseFlag == 0){
                Gimbal_Target.yaw_angle_target = YawAngleSave;
                CruiseFlag  = 1;
                ShootFlag   = 0;
                RunAwayFlag = 0;
                ControlFlag = 0;
            }
//            Cruise_Time_Between++;
//            if(Cruise_Time_Between > 10){
//                Gimbal_Target.yaw_angle_target = Gimbal_Target.yaw_angle_target + 2.0f;
//                Cruise_Time_Between=0;
//            }
            static int i=0;
            if(i==4000){
            Gimbal_Target.yaw_angle_target += 50;
            }
            else if(i>=8000){
            Gimbal_Target.yaw_angle_target -= 50;
            i=0;
            }
            i++;
            GimbalAngleLimit();
            GMYPositionPID.ref = Gimbal_Target.yaw_angle_target;
            GMYPositionPID.fdb = GMYawEncoder.ecd_angle;
        }break;
        case SHOOT_STATE:{
            if(ShootFlag == 0){
                Gimbal_Target.yaw_angle_target = YawAngleSave; 
                ShootFlag   = 1;
                CruiseFlag  = 0;
                RunAwayFlag = 0;
            }
            GimbalAngleLimit();
            GMYPositionPID.ref = Gimbal_Target.yaw_angle_target;
            GMYPositionPID.fdb = GMYawEncoder.ecd_angle;
        }break;
        case RUNAWAY_STATE:{
            if(RunAwayFlag == 0){
                Gimbal_Target.yaw_angle_target = YawAngleSave;
                RunAwayFlag=1;
                CruiseFlag=0;
            }
            GMYPositionPID.ref = YawAngleSave;
            GMYPositionPID.fdb = GMYawEncoder.ecd_angle;
        }break;
        case STOP_STATE:{
            
        }break;
        case CONTROL_STATE:{
            static int i=0;
            if(i==5000){
                Gimbal_Target.yaw_angle_target += 50;
            }
            else if(i>=10000){
                Gimbal_Target.yaw_angle_target -= 50;
                i=0;
            }
            i++;
            if(ControlFlag == 0){
                Gimbal_Target.yaw_angle_target = YawAngleSave;
                ControlFlag = 1;
                CruiseFlag  = 0;
                ShootFlag   = 0;
            }
            GimbalAngleLimit();
            GMYPositionPID.ref = Gimbal_Target.yaw_angle_target;
            GMYPositionPID.fdb = GMYawEncoder.ecd_angle;
        }break;
    }
    YawAngleSave = GMYawEncoder.ecd_angle;
}

void GMPitchControlLoop(void){
//    GMPPositionPID.kp = PITCH_POSITION_KP_DEFAULTS;
//    GMPPositionPID.ki = PITCH_POSITION_KI_DEFAULTS;
//    GMPPositionPID.kd = PITCH_POSITION_KD_DEFAULTS;
//    GMPSpeedPID.kp = PITCH_SPEED_KP_DEFAULTS;
//    GMPSpeedPID.ki = PITCH_SPEED_KI_DEFAULTS;
//    GMPSpeedPID.kd = PITCH_SPEED_KD_DEFAULTS;
//    static int i=0;
//    if(i==5000){
//        Gimbal_Target.pitch_angle_target = 10;
//    }
//    else if(i>=10000){
//        Gimbal_Target.pitch_angle_target = -10;
//        i=0;
//    }
//    i++;
    GimbalAngleLimit();
    GMPPositionPID.ref = Gimbal_Target.pitch_angle_target;
    GMPPositionPID.fdb = GMPitchEncoder.ecd_angle * GMPitchRamp.Calc(&GMPitchRamp);
    GMPPositionPID.Calc(&GMPPositionPID);
    GMPSpeedPID.ref = GMPPositionPID.output;
    GMPSpeedPID.fdb = get_imu_wy();
    GMPSpeedPID.Calc(&GMPSpeedPID);
}

void GMYawControlLoop(void){
    switch(GetWorkState()){
        case CRUISE_STATE:{
//            GMYPositionPID.kp = 8;
//            GMYPositionPID.ki = YAW_POSITION_KI_DEFAULTS;
//            GMYPositionPID.kd = YAW_POSITION_KD_DEFAULTS;
//            GMYSpeedPID.kp = 2;
//            GMYSpeedPID.ki = YAW_SPEED_KI_DEFAULTS;
//            GMYSpeedPID.kd = YAW_SPEED_KD_DEFAULTS;
        }break;
        case SHOOT_STATE:{
//            GMYPositionPID.kp = 15;
//            GMYPositionPID.ki = YAW_POSITION_KI_DEFAULTS;
//            GMYPositionPID.kd = YAW_POSITION_KD_DEFAULTS;
//            GMYSpeedPID.kp = 18;
//            GMYSpeedPID.ki = YAW_SPEED_KI_DEFAULTS;
//            GMYSpeedPID.kd = YAW_SPEED_KD_DEFAULTS;
        }break;
        case CONTROL_STATE:{
//            GMYPositionPID.kp = YAW_POSITION_KP_DEFAULTS;
//            GMYPositionPID.ki = YAW_POSITION_KI_DEFAULTS;
//            GMYPositionPID.kd = YAW_POSITION_KD_DEFAULTS;
//            GMYSpeedPID.kp = YAW_SPEED_KP_DEFAULTS;
//            GMYSpeedPID.ki = YAW_SPEED_KI_DEFAULTS;
//            GMYSpeedPID.kd = YAW_SPEED_KD_DEFAULTS;
        }break;
        default:{
//            GMYPositionPID.kp = 15;
//            GMYPositionPID.ki = YAW_POSITION_KI_DEFAULTS;
//            GMYPositionPID.kd = YAW_POSITION_KD_DEFAULTS;[`
            
//            GMYSpeedPID.kp = 18;
//            GMYSpeedPID.ki = YAW_SPEED_KI_DEFAULTS;
//            GMYSpeedPID.kd = YAW_SPEED_KD_DEFAULTS;
        }break;
    }


    GMYPositionPID.Calc(&GMYPositionPID);
    GMYSpeedPID.ref = GMYPositionPID.output;
//    GMYSpeedPID.ref = 0;
//    static int i=0;
//    if(i==4000){
//        GMYSpeedPID.ref = 50;
//    }
//    else if(i>=8000){
//        GMYSpeedPID.ref = -50;
//        i=0;
//    }
//    i++;
    
    GMYSpeedPID.fdb = get_imu_wz();
    GMYSpeedPID.Calc(&GMYSpeedPID);
}


void SetGimbalMotorOutput(void){

//    Set_Gimbal_Current(CAN1, -(int16_t)GMYPositionPID.output, -(int16_t)GMPPositionPID.output);
    Set_Gimbal_Current(CAN1, -(int16_t)GMYSpeedPID.output, -(int16_t)GMPSpeedPID.output);
}

void UpperMonitorControlLoop(void){
    UpperMonitor_Ctr_t cmd = GetUpperMonitorCmd();
    switch(cmd.gimbalMovingCtrType){
//        case GIMBAL_CMD_MOVETO:{            
//            
//            Gimbal_Target.yaw_angle_target   += cmd.d1;
//            Gimbal_Target.pitch_angle_target += cmd.d2;
//            ResetUpperMonitorCmd_d1_d2();
//        }break;
        case GIMBAL_CMD_MOVEBY:{
            Gimbal_Target.yaw_angle_target   += cmd.d1;
            Gimbal_Target.pitch_angle_target += cmd.d2;
        }break;
        default:{
            
        }break;
    }
    GimbalAngleLimit();
}

uint8_t RammerHeatControl(void){
    if ( Get_Sentry_HeatData() > HeatLimit - 2*ShootSpeed )
        return 1;
    else
        return 0;
}

void ShootControlLoop(void){
    if ( Get_Flag_AutoShoot() == 0 ){
        shooting = 0;
    }
    else{
        shooting = 1;
    }
    if ( Rammer.torque > 11000 ){
        Set_t_inversion(120);
    }
    if ( Get_t_inversion() > 0 ){
        RammerSpeedPID( ( int16_t ) -1000 );
        Set_Rammer_Current( CAN1, (int16_t)RAMMERSpeedPID.output );
        Set_t_inversion(Get_t_inversion() - 1);
    }
    else if ( shooting == 0 || RammerHeatControl() == 1 ){
        RammerSpeedPID( ( int16_t ) 0 );
        Set_Rammer_Current( CAN1, (int16_t)RAMMERSpeedPID.output );
    }
    else{
        RammerSpeedPID( ( int16_t ) 6000 );
        Set_Rammer_Current( CAN1, (int16_t)RAMMERSpeedPID.output );
    }

}

void RammerSpeedPID( int16_t TargetSpeed){
    RAMMERSpeedPID.kp  = RAMMER_SPEED_KP_DEFAULTS;
    RAMMERSpeedPID.ki  = RAMMER_SPEED_KI_DEFAULTS;
    RAMMERSpeedPID.kd  = RAMMER_SPEED_KD_DEFAULTS;
    RAMMERSpeedPID.ref = TargetSpeed;
    RAMMERSpeedPID.fdb = Rammer.speed;
    RAMMERSpeedPID.Calc(&RAMMERSpeedPID);
}

void ControtTaskInit(void){
    time_tick_1ms = 0;
    SetWorkState(PREPARE_STATE); 
    GMPitchRamp.SetScale(&GMPitchRamp, PREPARE_TIME_TICK_MS);
    GMYawRamp.SetScale(&GMYawRamp, PREPARE_TIME_TICK_MS);
    GMPitchRamp.ResetCounter(&GMPitchRamp);
    GMYawRamp.ResetCounter(&GMYawRamp);
    Gimbal_Target.pitch_angle_target = 0.0f;
    Gimbal_Target.yaw_angle_target   = 0.0f;
    RAMMERSpeedPID.Reset(&RAMMERSpeedPID);
    GMPPositionPID.Reset(&GMPPositionPID);
    GMPSpeedPID.Reset(&GMPSpeedPID);
    GMYPositionPID.Reset(&GMYPositionPID);
    GMYSpeedPID.Reset(&GMYSpeedPID);
}

void Monitor_Pitch_PID(void)
{
    printf("PKP:%3.2f,PKI:%1.2f,PKD:%3.2f,VKP:%3.2f,VKI:%1.2f,VKD:%3.2f\r\n",
    GMPPositionPID.kp,GMPPositionPID.ki,GMPPositionPID.kd,GMPSpeedPID.kp,GMPSpeedPID.ki,GMPSpeedPID.kd);
}
void Monitor_Yaw_PID(void)
{
    printf("PKP:%3.2f,PKI:%1.2f,PKD:%3.2f,VKP:%3.2f,VKI:%1.2f,VKD:%3.2f\r\n",
    GMYPositionPID.kp,GMYPositionPID.ki,GMYPositionPID.kd,GMYSpeedPID.kp,GMYSpeedPID.ki,GMYSpeedPID.kd);
}
void Monitor_Rammer_PID(void)
{
    printf("VKP:%3.2f,VKI:%1.2f,VKD:%3.2f\r\n",RAMMERSpeedPID.kp,RAMMERSpeedPID.ki,RAMMERSpeedPID.kd);
}
void GMP_PID_PLUS(int x,int y)
{
    switch(x){
    case 0x1:
        GMPPositionPID.kp += (float)y/100;
        break;
    case 0x2:
        GMPPositionPID.ki += (float)y/100;
        break;
    case 0x3:
        GMPPositionPID.kd += (float)y/100;
        break;
    case 0x4:
        GMPSpeedPID.kp += (float)y/100;
        break;
    case 0x5:
        GMPSpeedPID.ki += (float)y/100;
        break;
    case 0x6:
        GMPSpeedPID.kd += (float)y/100;
        break;
    default :
        ;
    }
    Monitor_Pitch_PID();
}
void GMP_PID_MIN(int x,int y)
{
    switch(x){
    case 0x1:
        GMPPositionPID.kp -= (float)y/100;
        break;
    case 0x2:
        GMPPositionPID.ki -= (float)y/100;
        break;
    case 0x3:
        GMPPositionPID.kd -= (float)y/100;
        break;
    case 0x4:
        GMPSpeedPID.kp -= (float)y/100;
        break;
    case 0x5:
        GMPSpeedPID.ki -= (float)y/100;
        break;
    case 0x6:
        GMPSpeedPID.kd -= (float)y/100;
        break;
    default :
        ;
    }
    Monitor_Pitch_PID();
}
void GMY_PID_PLUS(int x,int y)
{
    switch(x){
    case 0x1:
        GMYPositionPID.kp += (float)y/100;
        break;
    case 0x2:
        GMYPositionPID.ki += (float)y/100;
        break;
    case 0x3:
        GMYPositionPID.kd += (float)y/100;
        break;
    case 0x4:
        GMYSpeedPID.kp += (float)y/100;
        break;
    case 0x5:
        GMYSpeedPID.ki += (float)y/100;
        break;
    case 0x6:
        GMYSpeedPID.kd += (float)y/100;
        break;
    default :
        ;
    }
    Monitor_Yaw_PID();
}
void GMY_PID_MIN(int x,int y)
{
    switch(x){
    case 0x1:
        GMYPositionPID.kp -= (float)y/100;
        break;
    case 0x2:
        GMYPositionPID.ki -= (float)y/100;
        break;
    case 0x3:
        GMYPositionPID.kd -= (float)y/100;
        break;
    case 0x4:
        GMYSpeedPID.kp -= (float)y/100;
        break;
    case 0x5:
        GMYSpeedPID.ki -= (float)y/100;
        break;
    case 0x6:
        GMYSpeedPID.kd -= (float)y/100;
        break;
    default :
        ;
    }
    Monitor_Yaw_PID();
}

void RAMMER_PID_PLUS(int x,int y)
{
    switch(x){
    case 0x1:
        RAMMERSpeedPID.kp += (float)y/100;
        break;
    case 0x2:
        RAMMERSpeedPID.ki += (float)y/100;
        break;
    case 0x3:
        RAMMERSpeedPID.kd += (float)y/100;
        break;
    default :
        ;
    }
    Monitor_Rammer_PID();
}
void RAMMER_PID_MIN(int x,int y)
{
    switch(x){
    case 0x1:
        RAMMERSpeedPID.kp -= (float)y/100;
        break;
    case 0x2:
        RAMMERSpeedPID.ki -= (float)y/100;
        break;
    case 0x3:
        RAMMERSpeedPID.kd -= (float)y/100;
        break;
    default :
        ;
    }
    Monitor_Rammer_PID();
}
