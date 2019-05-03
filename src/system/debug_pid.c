#include "debug_pid.h"
#include "control_task.h"
#include "sys.h"
#include "usart3.h"

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
