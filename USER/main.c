#include "main.h"
#include "bsp.h"
#include "imu.h"
#include "beep.h" 
#include "usart3.h"
#include "delay.h"
#include "can_bus_task.h"
#include "gun.h"
#include "visualscope.h"
#include "control_task.h"

int main(void){
    BSP_Pre_Init();
    delay_ms(5000);
    if(Check_Sum > 1){
        printf("Check your monitors!\r\n");
        Sing_bad_case();
    }

    BSP_Init();

    while(1){
        printf("%8f, %8.3lf, %8.3lf\r\n", imu.rip.yaw, imu.rip.pit,imu.rip.rol);
//          printf("%8f\r\n", imu.rip.temp);
//        float x=0;
//        x=imu.rip.yaw-imu.rip.pit;
//        VisualScope(USART3, imu.rip.yaw, imu.rip.pit, x, 0);
        imu_main();
    }
}
