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
    delay_ms(5000);
    if(Check_Sum > 1){
        printf("Check your Monitors!\r\n");
        Sing_bad_case();
    }
    BSP_Init();

    while(1){
//        printf("%8f, %8.3lf, %8.3lf\r\n", imu.rip.yaw, imu.rip.pit,imu.rip.rol);
//        printf("%5.3f:,%5d:\r\n", imu.rip.gz,imu.raw.gz);
//      float x=0;
//      x=imu.rip.yaw-imu.rip.pit;
//        VisualScope(USART2, GMYPositionPID.fdb,GMYPositionPID.ref, GMYSpeedPID.fdb, GMYSpeedPID.ref);
        VisualScope(USART2, RAMMERSpeedPID.fdb,RAMMERSpeedPID.ref,0,0);
//        VisualScope(USART2, GMPPositionPID.fdb,GMPPositionPID.ref, GMPSpeedPID.fdb, GMPSpeedPID.ref);
        imu_main();
        delay_ms(2);
    }
}
