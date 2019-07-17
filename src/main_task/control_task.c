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


volatile static int16_t  FRICTION_WHEEL_MAX_DUTY = 1320;
volatile int16_t minipc_alive_count = 0;
volatile static float first_pit_angle = 0;
volatile static uint8_t pit_angle_limit_flag = 0;
void GimbalAngleLimit(void){
    if(Gimbal_Target.pitch_angle_target <= PITCH_MIN){
        Gimbal_Target.pitch_angle_target = PITCH_MIN;
        pit_angle_limit_flag = 1;
    }
    else if(Gimbal_Target.pitch_angle_target >= PITCH_MAX){
        Gimbal_Target.pitch_angle_target = PITCH_MAX;
        pit_angle_limit_flag = 1;
    }
    else{
        pit_angle_limit_flag = 0;
    }
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
            }
        }break;
        case CRUISE_STATE:{
            if(GetUpperMonitorOnline() == 1){
                workState = SHOOT_STATE;
            }
            if(GetControlMode() == REMOTE_CONTROL){
                workState = CONTROL_STATE;
            }
        }break;
        case SHOOT_STATE:{
            if(GetUpperMonitorOnline() == 0){
                workState = CRUISE_STATE;
            }
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
/**
  * @author         李运环
  * @brief          云台电机各状态任务
  * @data           2019.5.4
  * @param[in]      NULL
  * @retval         返回空
  */
void GMYawPitchModeSwitch(void){
    static int16_t  Cruise_Time_Between = 0;
    static uint8_t  pitch_dowm_flag = 0;
    static uint8_t  first_prepare_flag = 1;
    static uint8_t  first_cruise_flag = 1;
    switch(GetWorkState()){
        case PREPARE_STATE:{
            Gimbal_Target.yaw_angle_target = GET_YAW_ANGLE;
            if(first_prepare_flag == 1 && GMPitchEncoder.ecd_angle != 0){
                Gimbal_Target.pitch_angle_target = PITCH_INIT_ANGLE;
                first_pit_angle =  GMPitchEncoder.ecd_angle;
                first_prepare_flag = 0;
            };
        }break;
        case CRUISE_STATE:{
            if(first_cruise_flag == 1){
                Set_Flag(Gimble_ok);
                SetFrictionState(FRICTION_WHEEL_ON);
                first_cruise_flag = 0;
            };
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
            UpperMonitorControlLoop();
            Send_Gimbal_Info(1, 0, 0);
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
            Send_Gimbal_Info(0,1,Get_ChassisSpeed_Target());
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
    
    GMPPositionPID.ref = first_pit_angle + (Gimbal_Target.pitch_angle_target - first_pit_angle) * GMPitchRamp.Calc(&GMPitchRamp);
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

void RammerSpeedPID( int16_t TargetSpeed){
    RAMMERSpeedPID.ref = TargetSpeed;
    RAMMERSpeedPID.fdb = Rammer.speed;
    RAMMERSpeedPID.Calc(&RAMMERSpeedPID);
}

/**
  * @author         李运环
  * @brief          枪口热量控制
  * @date           2019.7.9
  * @param[in]      从裁判系统读取的枪口热量， 从裁判系统读取的实时射速
  * @retval         拨盘停止返回1，否则返回0
  */
#define SENTRY_HEAT_THRESHOLD   480
#define VELOCITY_THRESHOLD      30

int8_t heat_control(volatile int16_t const current_heat, volatile float const current_velocity){
//    static int8_t available_bullet_num = 0;
//    static int8_t launched_bullet_num = 0;
//    static float velocity_last = 0;
//    static float velocity_now = 0;
//    static int16_t heat_last = 0;
//    static int16_t heat_now = 480;
    static int8_t heat_control_flag = 0;
    
//    available_bullet_num = (SENTRY_HEAT_THRESHOLD - current_heat) / VELOCITY_THRESHOLD - 1;

//    velocity_last = velocity_now;
//    velocity_now = current_velocity;
//    
//    heat_last = heat_now;
//    heat_now = current_heat;
//    if(heat_last != heat_now){
//        launched_bullet_num = 0;           //热量更新，则已发射子弹清零
//    }
//    
//    if(velocity_now != velocity_last){     //发射数据更新
//        launched_bullet_num++;             //已发射子弹加一
//    }
//    if(launched_bullet_num >= available_bullet_num){         //已发射子弹数大于等于可发射子弹数
//        heat_control_flag = 1;             //拨盘停止
//        launched_bullet_num = 0;           //子弹清零
//    }
//    else{
//        heat_control_flag = 0;
//    }
//当热量数据更新频率较高时，可直接用下面的公式限制，简单有效
    if((SENTRY_HEAT_THRESHOLD - current_heat) <= 35){
        heat_control_flag = 1;
    }
    else{
        heat_control_flag = 0;
    }
    
    return heat_control_flag;
}

/**
  * @brief          拨弹电机控制
  * @author         李运环
   *@param[in]      NULL
  * @retval         返回空
  */
#define RAMMER_TORQUE_THRESHOLD  11000  //堵转扭矩
#define RAMMER_INVERSE_TIME_MS   1000   //反转时间
#define RAMMER_INVERSE_SPEED     -1000  //反转速度
#define RAMMER_NORMAL_SPEED      3000   //拨盘转速  转/min
void ShootControlLoop(void){
    volatile static int8_t  heat_control_flag = 0;
    volatile static int16_t rammer_speed      = 0;
    volatile static int16_t t_inversion       = 0;
    
    if ( Rammer.torque > RAMMER_TORQUE_THRESHOLD ){
        t_inversion = RAMMER_INVERSE_TIME_MS/2;
    }
    if ( t_inversion > 0 ){
        rammer_speed = RAMMER_INVERSE_SPEED;
        t_inversion--;
    }
    
    heat_control_flag = heat_control(Get_Sentry_HeatData(), Get_Sentry_BulletSpeed());
    
    if (Get_Flag(Shoot) == 0 || Get_Flag(Auto_aim_debug) == 1 || heat_control_flag == 1 || pit_angle_limit_flag == 1){
        rammer_speed = 0;
    }
    else{
        rammer_speed = RAMMER_NORMAL_SPEED;
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
  * @brief          摩擦轮开关控制，通过读取枚举FrictionWheelState_e变量判断状态，先让一个转，再让另一个转。
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
//            if(FrictionRamp2.IsOverflow(&FrictionRamp2)){
//                first_stop_friction = 1;
//                stop_flag = 1;
//                //此处添加拨盘开
//            }
        }
//        friction_wheel_speed_1 = FRICTION_WHEEL_MAX_DUTY;
//        friction_wheel_speed_2 = FRICTION_WHEEL_MAX_DUTY;
    }
    else if(GetFrictionState() == FRICTION_WHEEL_OFF){          //摩擦轮关
        //此处添加拨盘关
//        if(stop_flag == 1){
//            if(first_stop_friction == 1){
//                FrictionRamp1.ResetCounter(&FrictionRamp1);
//                FrictionRamp2.ResetCounter(&FrictionRamp2);
//                first_start_friction = 1;
//                first_stop_friction = 0;
//            }
//            friction_wheel_speed_2 = FRICTION_WHEEL_MAX_DUTY - (FRICTION_WHEEL_MAX_DUTY-1000)*FrictionRamp2.Calc(&FrictionRamp2);
//            if(FrictionRamp2.IsOverflow(&FrictionRamp2)){
//                friction_wheel_speed_1 = FRICTION_WHEEL_MAX_DUTY - (FRICTION_WHEEL_MAX_DUTY-1000)*FrictionRamp1.Calc(&FrictionRamp1);
//                if(FrictionRamp1.IsOverflow(&FrictionRamp1)){
//                    stop_flag = 0;
//                }
//            }
//        }
        friction_wheel_speed_1 = 1000;
        friction_wheel_speed_2 = 1000;
        first_start_friction = 1;
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
    if(minipc_alive_count++ == 1000){
        SetUpperMonitorOnline(0);
    }
    WorkStateFSM();
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
