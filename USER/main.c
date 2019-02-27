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
    usmart_dev.init(SystemCoreClock/1000000);
    delay_ms(5000);
    if(Check_Sum > 1){
        printf("Check your monitors!\r\n");
        Sing_bad_case();
    }
    BSP_Init();

    while(1){
//        printf("%8f, %8.3lf, %8.3lf\r\n", imu.rip.yaw, imu.rip.pit,imu.rip.rol);
//        printf("%8f\r\n", imu.rip.temp);
//      float x=0;
//      x=imu.rip.yaw-imu.rip.pit;
      VisualScope(USART2, GMYPositionPID.output, GMYSpeedPID.output, Gimbal_Target.yaw_angle_target, GMYawEncoder.ecd_angle);
      imu_main();
    }
}
