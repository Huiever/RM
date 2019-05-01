#ifndef _REOMTE_TASK_H_
#define _REOMTE_TASK_H_
#include "main.h"
#include "ramp.h"

#define PITCH_MAX                   0.0f
#define PITCH_MIN                   -40.0f


#define FRICTION_RAMP_TICK_COUNT    200                

#define REMOTE_CONTROLLER_STICK_OFFSET      1024u

#define STICK_TO_CHASSIS_SPEED_REF_FACT     0.50f
#define STICK_TO_PITCH_ANGLE_INC_FACT       0.004f
#define STICK_TO_YAW_ANGLE_INC_FACT         0.020f

#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
    val = min;\
}\
else if(val>=max)\
{\
    val = max;\
}\
else\
{\
    val = val;\
}

typedef struct {
    uint8_t start1;
    uint8_t start2;
    uint8_t cmdid;
    uint8_t data[4];
    uint8_t end1;
    uint8_t end2;
}package_t;


typedef __packed struct{
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int8_t s1;
    int8_t s2;
}Remote;

typedef __packed struct{
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t last_press_l;
    uint8_t last_press_r;
    uint8_t press_l;
    uint8_t press_r;
}Mouse;

typedef __packed struct{
    uint16_t v;
}Key;

typedef __packed struct{
    Remote rc;
    Mouse mouse;
    Key key;
}RC_Ctl_t;

typedef enum{
    REMOTE_CONTROL = 1,
    AUTO_CONTROL = 3,
    STOP = 2,
}ControlMode_e;

typedef struct{
    float pitch_angle_target;
    float yaw_angle_target;
    float pitch_speed_target;
    float yaw_speed_target;    
}Gimbal_Target_t;

typedef enum{
    FRICTION_WHEEL_OFF = 0,
    FRICTION_WHEEL_ON = 1,
}FrictionWheelState_e;

typedef enum{
    NOSHOOTING = 0,
    SHOOTING = 1,
}Shoot_State_e;

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

typedef struct{
    float pitch_angle_dynamic_ref;
    float yaw_angle_dynamic_ref;
    float pitch_angle_static_ref;
    float yaw_angle_static_ref;
    float pitch_speed_ref;
    float yaw_speed_ref;
}Gimbal_Ref_t;

extern Gimbal_Target_t Gimbal_Target;
extern int start_friction_flag;
extern RampGen_t          FrictionRamp1;
extern RampGen_t          FrictionRamp2;
extern int  FRICTION_WHEEL_MAX_DUTY;
extern int first_start_friction;

void RemoteTaskInit(void);
void GimbalAngleLimit(void);

FrictionWheelState_e GetFrictionState(void);
uint8_t Get_Flag_AutoShoot(void);
int16_t Get_t_inversion(void);
int16_t Get_MiniPC_Alive_Count(void);
void Set_Flag_AutoShoot(uint8_t flag);
void Reset_MiniPC_Alive_Count(void);
void MiniPC_Alive_Count_PlusPlus(void);
void Set_FRICTION_WHEEL_MAX_DUTY(uint32_t x);
void SetFrictionState(FrictionWheelState_e v);
void Set_t_inversion(int16_t t);

UpperMonitor_Ctr_t GetUpperMonitorCmd(void);
uint8_t GetUpperMonitorOnline(void);
void SetUpperMonitorOnline(uint8_t isOnline);
void UpperMonitorDataProcess(uint8_t *pData);
void ResetUpperMonitorCmd(void);
void ResetUpperMonitorCmd_d1_d2(void);
void RequestFinishFrictionSpeedUp(void);

ControlMode_e GetControlMode(void);
uint8_t Is_Control_State(void);
int16_t Get_ChassisSpeed_Target(void);
void RemoteDataProcess(uint8_t *pData);
void SetControlMode(Remote *rc);
void Reset_ChassisSpeed_Target(void);

void miniPC_task(void);
extern RC_Ctl_t           RC_CtrlData;
#endif
