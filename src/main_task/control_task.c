#include "control_task.h"
#include "pid_regulator.h"
#include "ramp.h"
#include "remote_task.h"
#include "flags.h"
#include "can_bus_task.h"
#include "imu.h"
#include "usart3.h"
#include "remote_task.h"
#include "main.h"
#include "gun.h"
#include "minipc_task.h"

PID_Regulator_t GMPPositionPID = GIMBAL_MOTOR_PITCH_POSITION_PID_DEFAULT;
PID_Regulator_t GMPSpeedPID    = GIMBAL_MOTOR_PITCH_SPEED_PID_DEFAULT;
PID_Regulator_t GMYPositionPID = GIMBAL_MOTOR_YAW_POSITION_PID_DEFAULT;
PID_Regulator_t GMYSpeedPID    = GIMBAL_MOTOR_YAW_SPEED_PID_DEFAULT;
PID_Regulator_t RAMMERSpeedPID = RAMMER_SPEED_PID_DEFAULT;

RampGen_t GMPitchRamp   = RAMP_GEN_DAFAULT;
RampGen_t GMYawRamp     = RAMP_GEN_DAFAULT;
RampGen_t FrictionRamp1 = RAMP_GEN_DAFAULT;
RampGen_t FrictionRamp2 = RAMP_GEN_DAFAULT;

WorkState_e lastWorkState = PREPARE_STATE;
WorkState_e workState     = PREPARE_STATE;
FrictionWheelState_e friction_wheel_state = FRICTION_WHEEL_OFF;

volatile static int16_t  t_inversion             = 0;
volatile static int16_t  FRICTION_WHEEL_MAX_DUTY = 1320;

/*
25    5000    1
17    5000    1/5
20    5000    1
*/
/*
1300   <22
1350   <26
*/
void GimbalAngleLimit(void){
    VAL_LIMIT(Gimbal_Target.pitch_angle_target, PITCH_MIN + 5.0f, PITCH_MAX-5.0f);
}

/**
  * @author         李运环
  * @brief          云台状态切换
  * @data           2019.5.4
  * @param[in]      NULL
  * @retval         返回空
  */
void WorkStateFSM(void){
    volatile static uint16_t time_tick_1ms = 0;
    lastWorkState = workState;
    switch(workState){
        case PREPARE_STATE:{
            if((++time_tick_1ms) > PREPARE_TIME_TICK_MS){
                workState = CRUISE_STATE;
                Set_Flag(Gimble_ok);
                SetFrictionState(FRICTION_WHEEL_ON);
            }
        }break;
        case CRUISE_STATE:{
            if(GetUpperMonitorOnline() == 1){
                workState = SHOOT_STATE;
            }
//            if(Get_Flag_In_RunAwayState() == 1){
//                workState = RUNAWAY_STATE;
//            }
            if(GetControlMode() == REMOTE_CONTROL){
                workState = CONTROL_STATE;
            }
        }break;
        case SHOOT_STATE:{
            if(GetUpperMonitorOnline() == 0){
                workState = CRUISE_STATE;
            }
            if(Get_Flag_In_RunAwayState() == 0){
                Send_Gimbal_Info(1,0,0);
            }
            
        }break;
        case RUNAWAY_STATE:{
//            if(Get_Flag_In_RunAwayState() == 0){
//                workState = CRUISE_STATE;
//            }
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

/**
  * @author         李运环
  * @brief          云台电机各状态任务
  * @data           2019.5.4
  * @param[in]      NULL
  * @retval         返回空
  */
void GMYawPitchModeSwitch(void){
    static uint8_t  CruiseFlag    = 0;
    static uint8_t  ShootFlag     = 0;
    static uint8_t  RunAwayFlag   = 0;
    static uint8_t  ControlFlag   = 0;
    static int16_t  Cruise_Time_Between = 0;
    static uint8_t  pitch_dowm_flag = 0;
    switch(GetWorkState()){
        case PREPARE_STATE:{
            Gimbal_Target.yaw_angle_target   = GET_YAW_ANGLE;
            Gimbal_Target.pitch_angle_target = PITCH_INIT_ANGLE;
        }break;
        case CRUISE_STATE:{
            if(CruiseFlag == 0){
                CruiseFlag  = 1;
                ShootFlag   = 0;
                RunAwayFlag = 0;
                ControlFlag = 0;
            }

#if DEBUG_YAW_PID == 0
            Cruise_Time_Between++;
            if(Cruise_Time_Between > 10){
                Gimbal_Target.yaw_angle_target += GIMBAL_YAW_CRUISE_DELTA;

                if(pitch_dowm_flag == 1){
                    Gimbal_Target.pitch_angle_target -= GIMBAL_PITCH_CRUISE_DELTA;
                }
                else{
                    Gimbal_Target.pitch_angle_target += GIMBAL_PITCH_CRUISE_DELTA;
                }
                
                if(Gimbal_Target.pitch_angle_target > PITCH_MAX-5){
                    pitch_dowm_flag = 1;
                }
                if(Gimbal_Target.pitch_angle_target < PITCH_MIN+5){
                    pitch_dowm_flag = 0;
                }
                Cruise_Time_Between=0;
            }

#endif

#if DEBUG_YAW_PID == 1
            static int i=0;
            if(i==4000){
                Gimbal_Target.yaw_angle_target += 50;
            }
            else if(i>=8000){
                Gimbal_Target.yaw_angle_target -= 50;
                i=0;
            }
            i++;
#endif
        }break;
        case SHOOT_STATE:{
            if(ShootFlag == 0){
                ShootFlag   = 1;
                CruiseFlag  = 0;
                RunAwayFlag = 0;
            }
        }break;
        case RUNAWAY_STATE:{
            if(RunAwayFlag == 0){
                RunAwayFlag=1;
                CruiseFlag=0;
            }
        }break;
        case STOP_STATE:{
            
        }break;
        case CONTROL_STATE:{
#if DEBUG_YAW_PID == 1
            static int i=0;
            if(i==5000){
                Gimbal_Target.yaw_angle_target += 50;
            }
            else if(i>=10000){
                Gimbal_Target.yaw_angle_target -= 50;
                i=0;
            }
            i++;
#endif
            if(ControlFlag == 0){
                ControlFlag = 1;
                CruiseFlag  = 0;
                ShootFlag   = 0;
            }
        }break;
    }

}

/**
  * @author         李运环
  * @brief          yaw电机控制
  * @data           2019.5.4
  * @param[in]      NULL
  * @retval         返回空
  */
void GMYawControlLoop(void){
    GMYPositionPID.ref = Gimbal_Target.yaw_angle_target;
    GMYPositionPID.fdb = GET_YAW_ANGLE;
    GMYPositionPID.Calc(&GMYPositionPID);
    
    GMYSpeedPID.ref = GMYPositionPID.output;

#if DEBUG_YAW_PID == 2
//    static int i=0;
//    if(i==4000){
//        GMYSpeedPID.ref = 100;
//    }
//    else if(i>=8000){
//        GMYSpeedPID.ref = -100;
//        i=0;
//    }
//    i++;
    GMYSpeedPID.ref = 0;
#endif

    GMYSpeedPID.fdb = GET_YAW_ANGULAR_SPEED;
    GMYSpeedPID.Calc(&GMYSpeedPID);
}

/**
  * @author         李运环
  * @brief          pitch电机控制
  * @latest         2019.5.4
  * @param[in]      NULL
  * @retval         返回空
  */
void GMPitchControlLoop(void){

#if DEBUG_PICTH_PID == 1
//    static int i=0;
//    if(i==5000){
//        Gimbal_Target.pitch_angle_target = 10;
//    }
//    else if(i>=10000){
//        Gimbal_Target.pitch_angle_target = -10;
//        i=0;
//    }
//    i++;
    Gimbal_Target.pitch_angle_target = 0;
#endif
    GimbalAngleLimit();
    
    GMPPositionPID.ref = Gimbal_Target.pitch_angle_target;
    GMPPositionPID.fdb = GMPitchEncoder.ecd_angle;
    GMPPositionPID.Calc(&GMPPositionPID);
    
    GMPSpeedPID.ref = GMPPositionPID.output;
//    GMPSpeedPID.ref = 0;
    GMPSpeedPID.fdb = GET_PITCH_ANGULAR_SPEED;
    GMPSpeedPID.Calc(&GMPSpeedPID);
}

void SetGimbalMotorOutput(void){
    Set_Gimbal_Current(CAN1, -(int16_t)GMYSpeedPID.output, -(int16_t)GMPSpeedPID.output);
//    Set_Gimbal_Current(CAN1, -(int16_t)GMYSpeedPID.output, 0);
//    Set_Gimbal_Current(CAN1, 0, -(int16_t)GMPSpeedPID.output);
}

/**
  * @brief          minipc控制回路
  * @author         李运环
  * @data           2019.5.4
  * @param[in]      NULL
  * @retval         返回空
  */
void UpperMonitorControlLoop(void){
    UpperMonitor_Ctr_t cmd = GetUpperMonitorCmd();
    switch(cmd.gimbalMovingCtrType){
        case GIMBAL_CMD_MOVEBY:{
            Gimbal_Target.yaw_angle_target   += cmd.d1;
            Gimbal_Target.pitch_angle_target += cmd.d2;
        }break;
        case GIMBAL_CMD_MOVETO:{
            Gimbal_Target.yaw_angle_target    = cmd.d1;
            Gimbal_Target.pitch_angle_target  = cmd.d2;
        }break;
        default:{
            
        }break;
    }
}

void Set_t_inversion(int16_t t){
    t_inversion = t;
}

int16_t Get_t_inversion(void){
    return t_inversion;
}

void RammerSpeedPID( int16_t TargetSpeed){
    RAMMERSpeedPID.ref = TargetSpeed;
    RAMMERSpeedPID.fdb = Rammer.speed;
    RAMMERSpeedPID.Calc(&RAMMERSpeedPID);
}

/**
  * @author         李运环
  * @brief          枪口热量控制
  * @date           2019.4.17
  * @param[in]      从裁判系统读取的枪口热量， 从裁判系统读取的实时射速
  * @retval         返回空
  */
#define SENTRY_HEAT_THRESHOLD   480

void heat_control(volatile int16_t const current_heat, volatile float const current_velocity){
    static int8_t available_bullet_num = 0;
    static int8_t velocity_threshold = 30;
    static int8_t launched_bullet_num = 0;
    static float velocity_last = 0;
    static float velocity_now = 0;
    static int8_t shoot_flag = 0;
    
    available_bullet_num = (SENTRY_HEAT_THRESHOLD - current_heat) / velocity_threshold - 1;

    velocity_last = velocity_now;
    velocity_now = current_velocity;          

    if(velocity_now != velocity_last){     //发射数据更新
        launched_bullet_num++;             //已发射子弹加一
    }
    if(launched_bullet_num >= available_bullet_num){         //已发射子弹数大于等于可发射子弹数
        shoot_flag = 0;                //拨盘停止
        launched_bullet_num = 0;       //子弹清零
    }
    else{
        shoot_flag = 1;
    }
    
    if(1 == shoot_flag && 1 == Get_Flag(Shoot)){
        Set_Flag(Shoot);
    }
    else{
        Reset_Flag(Shoot);
    }
    
    if((SENTRY_HEAT_THRESHOLD - current_heat) <= 35){
        Reset_Flag(Shoot);
    }
}

/**
  * @brief          拨弹电机控制
  * @author         李运环
   *@param[in]      NULL
  * @retval         返回空
  */
void ShootControlLoop(void){
    volatile static int16_t rammer_speed = 0;

    if ( Rammer.torque > 11000 ){
        Set_t_inversion(120);
    }
    if ( Get_t_inversion() > 0 ){
        rammer_speed = -1000;
        Set_t_inversion(Get_t_inversion() - 1);
    }
    
    heat_control(Get_Sentry_HeatData(), Get_Sentry_BulletSpeed());
    
    if ( Get_Flag(Shoot) == 0){
        rammer_speed = 0;
    }
    else if(Get_Flag_In_RunAwayState() == 0){
        rammer_speed = 3000;
    }
    else{
        rammer_speed = 2000;   //边跑边打降低射击频率
    }
    
    if(Get_Flag(Auto_aim_debug) == 1){
        rammer_speed = 0;
    }
    RammerSpeedPID(rammer_speed);
    Set_Rammer_Current(CAN1, (int16_t)RAMMERSpeedPID.output);
}

/**
  * @brief          摩擦轮初始化
  * @author         李运环
   *@param[in]      NULL
  * @retval         返回空
  */
void friction_init(void){
    FrictionRamp1.SetScale(&FrictionRamp1, FRICTION_RAMP_TICK_COUNT);
    FrictionRamp1.ResetCounter(&FrictionRamp1);
    FrictionRamp2.SetScale(&FrictionRamp2, FRICTION_RAMP_TICK_COUNT);
    FrictionRamp2.ResetCounter(&FrictionRamp2);
    SetFrictionState(FRICTION_WHEEL_OFF);
}

/**
  * @brief          摩擦轮开关控制，通过读取枚举FrictionWheelState_e变量判断状态，先让一个（停）转，再让另一个（停）转。
  * @author         李运环
  * @param[in]      NULL
  * @retval         返回空
  */

void friction_control(void){
    static int first_start_friction = 1;
    static int first_stop_friction = 0;
    static int stop_flag = 0;
    static int friction_wheel_speed_1 = 1000;
    static int friction_wheel_speed_2 = 1000;
    if(GetFrictionState() == FRICTION_WHEEL_ON){                //摩擦轮开
        if(first_start_friction == 1){
            FrictionRamp1.ResetCounter(&FrictionRamp1);
            FrictionRamp2.ResetCounter(&FrictionRamp2);
            first_start_friction = 0;
        }
        friction_wheel_speed_1 = 1000 + (FRICTION_WHEEL_MAX_DUTY-1000)*FrictionRamp1.Calc(&FrictionRamp1);
        if(FrictionRamp1.IsOverflow(&FrictionRamp1)){
            friction_wheel_speed_2 = 1000 + (FRICTION_WHEEL_MAX_DUTY-1000)*FrictionRamp2.Calc(&FrictionRamp2);
            if(FrictionRamp2.IsOverflow(&FrictionRamp2)){
                first_stop_friction = 1;
                stop_flag = 1;
                //此处添加拨盘开
            }
        }
    }
    else if(GetFrictionState() == FRICTION_WHEEL_OFF){          //摩擦轮关
        //此处添加拨盘关
        if(stop_flag == 1){
            if(first_stop_friction == 1){
                FrictionRamp1.ResetCounter(&FrictionRamp1);
                FrictionRamp2.ResetCounter(&FrictionRamp2);
                first_start_friction = 1;
                first_stop_friction = 0;
            }
            friction_wheel_speed_2 = FRICTION_WHEEL_MAX_DUTY - (FRICTION_WHEEL_MAX_DUTY-1000)*FrictionRamp2.Calc(&FrictionRamp2);
            if(FrictionRamp2.IsOverflow(&FrictionRamp2)){
                friction_wheel_speed_1 = FRICTION_WHEEL_MAX_DUTY - (FRICTION_WHEEL_MAX_DUTY-1000)*FrictionRamp1.Calc(&FrictionRamp1);
                if(FrictionRamp1.IsOverflow(&FrictionRamp1)){
                    stop_flag = 0;
                }
            }
        }
    }
    SetFrictionWheelSpeed_1(friction_wheel_speed_1);
    SetFrictionWheelSpeed_2(friction_wheel_speed_2);
}

void Set_FRICTION_WHEEL_MAX_DUTY( uint32_t x ){
    FRICTION_WHEEL_MAX_DUTY = x;
}

FrictionWheelState_e GetFrictionState(){
    return friction_wheel_state;
}

void SetFrictionState(FrictionWheelState_e v){
    friction_wheel_state = v;
}

/**
  * @author         李运环
  * @brief          云台控制任务初始化
  * @date           2019.5.4
  * @param[in]      NULL
  * @retval         返回空
  */
void ControtTaskInit(void){
    SetWorkState(PREPARE_STATE); 
    GMPitchRamp.SetScale(&GMPitchRamp, PREPARE_TIME_TICK_MS);
    GMYawRamp.SetScale(&GMYawRamp, PREPARE_TIME_TICK_MS);
    GMPitchRamp.ResetCounter(&GMPitchRamp);
    GMYawRamp.ResetCounter(&GMYawRamp);
    RAMMERSpeedPID.Reset(&RAMMERSpeedPID);
    GMPPositionPID.Reset(&GMPPositionPID);
    GMPSpeedPID.Reset(&GMPSpeedPID);
    GMYPositionPID.Reset(&GMYPositionPID);
    GMYSpeedPID.Reset(&GMYSpeedPID);
    friction_init();
}

/**
  * @author         李运环
  * @brief          云台控制任务主函数，在timer6中以1KHz的周期运行
  * @date           2019.5.4
  * @param[in]      NULL
  * @retval         返回空
  */
void Control_Task(void){
    volatile static uint8_t time_tick = 0;
    WorkStateFSM();
    if(GetWorkState() == SHOOT_STATE){
        UpperMonitorControlLoop();
    }
    GMYawPitchModeSwitch();
    GMYawControlLoop();
    GMPitchControlLoop();
#if Monitor_GM_Encoder == 0 && DISABLE_GIMBLA_OUTPUT == 0
    SetGimbalMotorOutput();
#endif
    if((++time_tick) % 2 == 0){
        friction_control();
        ShootControlLoop();
        time_tick = 0;
    }
    else{
        Send_Gimbal_Info(GetUpperMonitorOnline(),Is_Control_State(),Get_ChassisSpeed_Target());
    }
}
