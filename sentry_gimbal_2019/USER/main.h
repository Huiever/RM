#ifndef _MAIN_H_
#define _MAIN_H_

#include "flags.h"

/* Specify the prepare time(ms) */
#define PREPARE_TIME_MS     4000

/* Specify the robot
   So you can use '#if #elif #endif' to specify the parameter for the same kind of robot */
#define INFANTRY            2

/* Adjust the Kp and Ki of IMU module
   This can change the convergence speed of the IMU output */
#define IMU_Kp              8.0f
#define IMU_Ki              0.01f
#define ELLIPSOID_FIT 			1 //Õ÷«Úƒ‚∫œ

/* Choose to monitor the output of IMU */
#define Monitor_IMU_Angle   1
#define Monitor_IMU_Accel   0
#define Monitor_IMU_Mag     0
/* Choose to monitor the output of remoter */
#define Monitor_Remoter     0

/* Monitor the encoder offset of pitch/yaw motor */
#define Monitor_GM_Encoder  0

/* Monitor the encoder offset of rammer motor */
#define Monitor_rammer      0

/* Change the encoder offset of pitch/yaw motor */
#define Pit_Encoder_Offset  5030
#define Yaw_Encoder_Offset  3090

/* Change the max duty of friction wheel,
   which is the max speed of friction wheel */
#define Friction_Max_Duty   1200

/* You can only monitor one kind of parameter as recommended
   So we need to check the sum of "monitors" 
	 @usage: when adding a monitor macro definition,
	         you must add it here. */
#define Check_Sum           Monitor_IMU_Angle + Monitor_IMU_Accel + \
                            Monitor_Remoter + Monitor_GM_Encoder +  \
														Monitor_rammer+Monitor_IMU_Mag

#endif 
