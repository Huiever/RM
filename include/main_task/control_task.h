#ifndef _CONTROL_TASK_H_
#define _CONTROL_TASK_H_

#include "main.h"
#include "pid_regulator.h"
#include "can_bus_task.h"

#define PITCH_INIT_ANGLE            -20.0f
#define YAW_INIT_ANGLE              0.0f

#define PREPARE_TIME_TICK_MS      4000
#define YAW_POSITION_KP_DEFAULTS  10
#define YAW_POSITION_KI_DEFAULTS  0
#define YAW_POSITION_KD_DEFAULTS  0

#define YAW_SPEED_KP_DEFAULTS  10
#define YAW_SPEED_KI_DEFAULTS  0
#define YAW_SPEED_KD_DEFAULTS  0

#define PITCH_POSITION_KP_DEFAULTS  25
#define PITCH_POSITION_KI_DEFAULTS  0.3
#define PITCH_POSITION_KD_DEFAULTS  0

#define PITCH_SPEED_KP_DEFAULTS  20
#define PITCH_SPEED_KI_DEFAULTS  0.3
#define PITCH_SPEED_KD_DEFAULTS  0

#define RAMMER_SPEED_KP_DEFAULTS  18
#define RAMMER_SPEED_KI_DEFAULTS  0
#define RAMMER_SPEED_KD_DEFAULTS  0 

#define GIMBAL_YAW_CRUISE_DELTA   4.0f
#define GIMBAL_PITCH_CRUISE_DELTA 0.5f

//#define GET_YAW_ANGLE()           get_yaw_angle()
//#define GET_YAW_ANGLE()           imu_yaw_angle
#define GET_YAW_ANGLE()           GMYawEncoder.ecd_angle
#define GET_YAW_ANGULAR_SPEED     imu_yaw_angular_speed
#define GET_PITCH_ANGLE           GMPitchEncoder.ecd_angle
#define GET_PITCH_ANGULAR_SPEED   -imu_pitch_angular_speed

typedef enum{
    PREPARE_STATE,
    CRUISE_STATE,
    SHOOT_STATE,
    RUNAWAY_STATE,
    STOP_STATE,
    CONTROL_STATE,
}WorkState_e;

#define GIMBAL_MOTOR_PITCH_POSITION_PID_DEFAULT \
{\
    0,\
    0,\
    {0,0},\
    PITCH_POSITION_KP_DEFAULTS,\
    PITCH_POSITION_KI_DEFAULTS,\
    PITCH_POSITION_KD_DEFAULTS,\
    0,\
    0,\
    0,\
    4900,\
    1000,\
    1500,\
    0,\
    4900,\
    0,\
    0,\
    0,\
    &PID_Calc,\
    &PID_Reset,\
}

#define GIMBAL_MOTOR_PITCH_SPEED_PID_DEFAULT \
{\
    0,\
    0,\
    {0,0},\
    PITCH_SPEED_KP_DEFAULTS,\
    PITCH_SPEED_KI_DEFAULTS,\
    PITCH_SPEED_KD_DEFAULTS,\
    0,\
    0,\
    0,\
    4900,\
    1000,\
    1500,\
    0,\
    4900,\
    0,\
    0,\
    0,\
    &PID_Calc,\
    &PID_Reset,\
}

#define GIMBAL_MOTOR_YAW_POSITION_PID_DEFAULT \
{\
    0,\
    0,\
    {0,0},\
    YAW_POSITION_KP_DEFAULTS,\
    YAW_POSITION_KI_DEFAULTS,\
    YAW_POSITION_KD_DEFAULTS,\
    0,\
    0,\
    0,\
    4900,\
    1000,\
    1500,\
    0,\
    5000,\
    0,\
    0,\
    0,\
    &PID_Calc,\
    &PID_Reset,\
}

#define GIMBAL_MOTOR_YAW_SPEED_PID_DEFAULT \
{\
    0,\
    0,\
    {0,0},\
    YAW_SPEED_KP_DEFAULTS,\
    YAW_SPEED_KI_DEFAULTS,\
    YAW_SPEED_KD_DEFAULTS,\
    0,\
    0,\
    0,\
    4900,\
    1000,\
    1500,\
    0,\
    4900,\
    0,\
    0,\
    0,\
    &PID_Calc,\
    &PID_Reset,\
}

#define RAMMER_SPEED_PID_DEFAULT \
{\
    0,\
    0,\
    {0,0},\
    RAMMER_SPEED_KP_DEFAULTS,\
    RAMMER_SPEED_KI_DEFAULTS,\
    RAMMER_SPEED_KD_DEFAULTS,\
    0,\
    0,\
    0,\
    4900,\
    1000,\
    1500,\
    0,\
    10000,\
    0,\
    0,\
    0,\
    &PID_Calc,\
    &PID_Reset,\
}


extern PID_Regulator_t GMPPositionPID ;
extern PID_Regulator_t GMPSpeedPID    ;
extern PID_Regulator_t GMYPositionPID ;
extern PID_Regulator_t GMYSpeedPID    ;
extern PID_Regulator_t RAMMERSpeedPID ;

void Control_Task(void);
void ControtTaskInit(void);
void WorkStateFSM(void);
void SetWorkState(WorkState_e state);
WorkState_e GetWorkState(void);

void SetGimbalMotorOutput(void);
void GMYawControlLoop(void);
void GMPitchControlLoop(void);
void GimbalYawControlModeSwitch(void);

void UpperMonitorControlLoop(void);
void ShootControlLoop(void);
void RammerControlLoop(void);
void RammerSpeedPID( int16_t TargetSpeed);
void BigSymbolRecgShootControl(void);
uint8_t RammerHeatControl(void);

extern int8_t available_bullet_num ;
extern int8_t launched_bullet_num ;
#endif
