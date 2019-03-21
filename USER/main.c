#include "main.h"
#include "bsp.h"
#include "imu.h"
#include "gun.h"
#include "beep.h" 
#include "usart2.h"
#include "usart3.h"
#include "delay.h"
#include "usmart.h"
#include "remote_task.h"
#include "can_bus_task.h"
#include "control_task.h"
#include "visualscope.h"

int main(void){
    BSP_Pre_Init();
    usmart_dev.init(SystemCoreClock/1000000);//使用的是timer4
    delay_ms(1000);
    if(Check_Sum > 1){
        printf("Check your Monitors!\r\n");
        Sing_bad_case();
    }
    BSP_Init();

    while(1){
//        VisualScope(USART2, GMYPositionPID.fdb,GMYPositionPID.ref, GMYSpeedPID.fdb, GMYSpeedPID.ref);
//        VisualScope(USART2, RAMMERSpeedPID.fdb,RAMMERSpeedPID.ref,GMYPositionPID.fdb,GMYPositionPID.ref);
//        VisualScope(USART2, GMPPositionPID.fdb,GMPPositionPID.ref, GMPSpeedPID.fdb, GMPSpeedPID.ref);
        imu_main();
        delay_ms(2);
    }
}
