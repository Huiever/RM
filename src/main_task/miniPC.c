//#include "main.h"
//#include "miniPC.h"
//#include "control_task.h"
//#include "remote_task.h"
//#include "can_bus_task.h"
//#include "sys.h"
//#include "beep.h"
//#include "led.h"
//#include "usart6.h"


//static int PC_online = 0;
//static package_t package = { 0x0a,0x0d,NO_CMD,0x00,0x00,0x00,0x00,0x0d,0x0a };
//UpperMonitor_Ctr_t upperMonitorCmd = { 0,NO_CMD,0.0,0.0 };

//void UpperMonitorDataProcess(uint8_t *pData) {
//    upperMonitorCmd.cmdid = pData[0];
//    upperMonitorCmd.d1 = *((int16_t *)(pData + 1));
//    upperMonitorCmd.d2 = *((int16_t *)(pData + 3));
//    switch (pData[0]) {
//        //不涉及控制，内部处理
//        case REQUEST_PITCH: {
//            miniPC_ACK_status();
//        }break;

//        case ACK:{
//            PC_online = 1;
//        }break;
//         case START_UPPER_MONITOR_CTR:{
//            PC_online = 1;
//        }break;

////        case GIMBAL_MOVEBY: {
////            Gimbal_Target.yaw_angle_target   += upperMonitorCmd.d1 * 0.001;
////            Gimbal_Target.pitch_angle_target += upperMonitorCmd.d2 * 0.001;
////        }break;

//        case START_FRICTION:{
//            SetFrictionState(FRICTION_WHEEL_ON);
//            upperMonitorCmd.startFriction = 1;
//        }break;
//            
//        case STOP_FRICTION:{
//            SetFrictionState(FRICTION_WHEEL_OFF);
//            upperMonitorCmd.startFriction = 0;
//        }break;
//        
//        case START_SHOOTING:{
//            Set_Flag_AutoShoot(1);
//        }break;
//        
//        case STOP_SHOOTING:{
//            Set_Flag_AutoShoot(0);
//        }break;
//        
//        case REQUEST_CURR_STATE:{
//            
//        }break;
//        
//        case EXIT_UPPER_MONITOR_CTR:{
//            PC_online = 0;
//        }break;

//        default: {
//        }break;
//    }
//}

//UpperMonitor_Ctr_t GetUpperMonitorCmd(void) {//逐次递减
//    UpperMonitor_Ctr_t c = upperMonitorCmd;
//    upperMonitorCmd.d1 *= 0.8;
//    upperMonitorCmd.d2 *= 0.8;
//    return c;
//}

//void miniPC_BS_Start(int8_t pitch_angle) {
////    package.cmdid = 
////     = 1;      //level
////     = pitch_angle;
////    USART6_Print(minipcTransmitData, 9);
//}

//void miniPC_BS_end(int8_t pitch_angle) {
////     = 'c';
////    = 1;
////     = pitch_angle;
////    USART6_Print(minipcTransmitData, 9);
//}

//void miniPC_BS_normal(int8_t pitch_angle) {
//    package.start1 = 0x0a;
//    package.start2 = 0x0d;
//    package.cmdid = NO_CMD;
//    package.data[0] = pitch_angle;
//    USART6_Print((uint8_t*)&package, 9);
//}

//void miniPC_HS_ACK(void) {
//    package.cmdid = ACK;
//    USART6_Print((uint8_t*)&package, 9);
//}

//void miniPC_HS_SYN(void) {
//    package.cmdid = SYN;
//    USART6_Print((uint8_t*)&package, 9);
//}

//void miniPC_ACK_status(void){
//    package.cmdid = ACK_STATUS;
//    package.data[0] = 0;    //0自瞄
//    package.data[1] = 0x01; //当前等级
//    package.data[2] = (int8_t)GET_PITCH_ANGLE;
//    USART6_Print((uint8_t*)&package, 9);
//}

//void miniPC_task(void){//仅执行一次，无时延要求，在主函数中参与循环
//    static int lastPCState = 0;
//    if(PC_online && lastPCState == 0){
//        lastPCState = 1;
//        Sing_miniPC_online();
//    }
//    else if(PC_online == 0 && lastPCState == 0){    //发起握手
//        miniPC_HS_SYN();
//    }
//}

//uint8_t GetUpperMonitorOnline(void){
//    return PC_online;
//}
//void USART6_IRQHandler(void) {
//    if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET) {
//        static unsigned char rx_buffer[12];
//        static uint8_t receivedBytes = 0;
//        uint8_t tmp = USART_ReceiveData(USART6);
//        printf("%x",tmp);
//        switch (receivedBytes) {
//        case 0: {
//            if (tmp == 0x0a) {
//                receivedBytes++;
//            }
//            else {
//                receivedBytes = 0;
//            }
//            break;
//        }
//        case 1: {
//            if (tmp == 0x0d) {
//                receivedBytes++;
//            }
//            else {
//                receivedBytes = 0;
//            }
//            break;
//        }
//        case 2: {
//            receivedBytes++;
//            rx_buffer[0] = tmp;
//            break;
//        }
//        case 3: {
//            receivedBytes++;
//            rx_buffer[1] = tmp;
//            break;
//        }
//        case 4: {
//            receivedBytes++;
//            rx_buffer[2] = tmp;
//            break;
//        }
//        case 5: {
//            receivedBytes++;
//            rx_buffer[3] = tmp;
//            break;
//        }
//        case 6: {
//            receivedBytes++;
//            rx_buffer[4] = tmp;
//            break;
//        }
//        case 7: {
//            if (tmp == 0x0d) {
//                receivedBytes++;
//            }
//            else {
//                receivedBytes = 0;
//            }
//            break;
//        }
//        case 8: {
//            if (tmp == 0x0a) {
//                UpperMonitorDataProcess(rx_buffer);
//                LED1 = ~LED1;
//            }
//            receivedBytes = 0;
//            break;
//        }
//        default:
//            receivedBytes = 0;
//        }
//    }
//}

//void miniPC_Init(void) {
//    USART6_Init(115200);
//}
