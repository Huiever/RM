#include "minipc_task.h"
#include "sys.h"
#include "usart6.h"
#include "remote_task.h"
#include "ramp.h"
#include "gun.h"
#include "control_task.h"
#include "stdio.h"
#include "beep.h"

volatile uint8_t upperMonitorOnline = 0;
volatile static  UpperMonitor_Ctr_t upperMonitorCmd = {0,GIMBAL_CMD_STOP,0,0};
volatile static  package_t package = { 0x0a,0x0d,0xff,0x00,0x00,0x00,0x00,0x0d,0x0a };

void miniPC_ACK_status(void){
    package.cmdid = 0x11;
    package.data[0] = 0xff; 
    package.data[1] = 0xff;
    package.data[2] = GET_PITCH_ANGLE;
    USART6_Print((uint8_t*)&package, 9);
}
void miniPC_HS_SYN(void) {
    package.cmdid = 0x09;
    USART6_Print((uint8_t*)&package, 9);
}

void miniPC_task(void){//��ִ��һ�Σ���ʱ��Ҫ�����������в���ѭ��
    static int lastPCState = 0;
    if(Get_Flag(PC_ack) == 1 && lastPCState == 0){
        lastPCState = 1;
        Sing_miniPC_online();
    }
    else if(Get_Flag(PC_ack) == 0){    //��������
        miniPC_HS_SYN();
    }
}

void UpperMonitorDataProcess(volatile uint8_t *pData){
    static const uint8_t START_UPPER_MONITOR_CTR = 0x00;
    static const uint8_t SEND_STATUS             = 0x01;
    static const uint8_t GIMBAL_MOVEBY           = 0x02;
    static const uint8_t GIMBAL_TURN_BACK        = 0x03;
    static const uint8_t START_FRICTION          = 0x04;
    static const uint8_t STOP_FRICTION           = 0x05;
    static const uint8_t START_SHOOTING          = 0x06;
    static const uint8_t STOP_SHOOTING           = 0x07;
    static const uint8_t REQUEST_CURR_STATE      = 0x08;
    static const uint8_t ACK                     = 0x10;
    static const uint8_t EXIT_UPPER_MONITOR_CTR  = 0x12;

    int16_t d1 = *((int16_t *)(pData + 1));
    int16_t d2 = *((int16_t *)(pData + 3));

    if(upperMonitorOnline){
        switch (pData[0]){
            case SEND_STATUS:{
                miniPC_ACK_status();
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
                SetFrictionState(FRICTION_WHEEL_ON);
                upperMonitorCmd.startFriction = 1;
            }break;
                
            case STOP_FRICTION:{
                SetFrictionState(FRICTION_WHEEL_OFF);
                upperMonitorCmd.startFriction = 0;
            }break;
            
            case START_SHOOTING:{
                Set_Flag(Shoot);
            }break;
            
            case STOP_SHOOTING:{
                Reset_Flag(Shoot);
            }break;
            
            case REQUEST_CURR_STATE:{
                
            }break;
            
            case EXIT_UPPER_MONITOR_CTR:{
                SetUpperMonitorOnline(0);
            }break;

            default:{
                ;
            }break;
        }
    }
    else if(pData[0] == START_UPPER_MONITOR_CTR){
                SetUpperMonitorOnline(1);
    }
    else if(pData[0] ==  ACK){
        Set_Flag(PC_ack);
    }
    else{
        ;
    }
}

UpperMonitor_Ctr_t GetUpperMonitorCmd(void){
    return upperMonitorCmd;
}

void SetUpperMonitorOnline(uint8_t isOnline){
    upperMonitorOnline = isOnline;
}

uint8_t GetUpperMonitorOnline(void){
    return upperMonitorOnline;
}
