#ifndef _MAIN_H_
#define _MAIN_H_

#include "flags.h"

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

/* Specify the prepare time(ms) */
#define PREPARE_TIME_MS             4000

/* Calibrate The Snail 430-R Racing ESC */
#define Calibrate_Snail_ESC         0

/* Choose to monitor the output of IMU */
#define Monitor_IMU_Angle           0
#define Monitor_IMU_Gyro            0
#define Monitor_IMU_Mag             0

/* Choose to monitor the output of remoter */
#define Monitor_Remoter             0

/* Debug pid parameters  0          1          2
                        null     position    speed */
#define DEBUG_YAW_PID               0
#define DEBUG_PITCH_PID             2

/* Monitor the encoder offset of pitch/yaw motor */
#define Monitor_GM_Encoder          0

/* disable yaw pitch move*/
#define DISABLE_GIMBLA_OUTPUT       0

/* Monitor the encoder offset of rammer motor */
#define Monitor_rammer              0

/* Change the encoder offset of pitch/yaw motor */
#define Pit_Encoder_Offset          1873

/* Change the max duty of friction wheel,
   which is the max speed of friction wheel */
#define Friction_Max_Duty           1200

/* You can only monitor one kind of parameter as recommended
   So we need to check the sum of "monitors" 
     @usage: when adding a monitor macro definition,
             you must add it here. */
#define Check_Sum                   Monitor_IMU_Angle + Monitor_IMU_Gyro + \
                                    Monitor_Remoter + Monitor_GM_Encoder +  \
                                    Monitor_rammer+Monitor_IMU_Mag

#endif
