#ifndef _REOMTE_TASK_H_
#define _REOMTE_TASK_H_
#include "main.h"
#include "ramp.h"

#define REMOTE_CONTROLLER_STICK_OFFSET      1024u

#define STICK_TO_CHASSIS_SPEED_REF_FACT     0.50f
#define STICK_TO_PITCH_ANGLE_INC_FACT       0.004f
#define STICK_TO_YAW_ANGLE_INC_FACT         0.020f

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
    DEBUG_SHOOT = 2,
}ControlMode_e;

typedef struct{
    float pitch_angle_target;
    float yaw_angle_target;
    float pitch_speed_target;
    float yaw_speed_target;    
}Gimbal_Target_t;


typedef enum{
    NOSHOOTING = 0,
    SHOOTING = 1,
}Shoot_State_e;

typedef struct{
    float pitch_angle_dynamic_ref;
    float yaw_angle_dynamic_ref;
    float pitch_angle_static_ref;
    float yaw_angle_static_ref;
    float pitch_speed_ref;
    float yaw_speed_ref;
}Gimbal_Ref_t;

extern Gimbal_Target_t Gimbal_Target;
extern RC_Ctl_t        RC_CtrlData;

ControlMode_e GetControlMode(void);
uint8_t Is_Control_State(void);
int16_t Get_ChassisSpeed_Target(void);
void RemoteDataProcess(uint8_t *pData);
void SetControlMode(Remote *rc);
void Reset_ChassisSpeed_Target(void);

#endif
