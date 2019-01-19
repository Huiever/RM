#ifndef _FLAGS_H_
#define _FLAGS_H_

#include <inttypes.h>
 
typedef enum{
	Shoot,            // shoot(1) or not(0) : shooting
	Shoot_mode,       // single shoot(1) or continuous shoot(0) : ShootMode
	Fric_accel_ok,    // friction accelerate complete(1) or not(0) : Frion_Flag
	Shoot_command,    // mouse left press down(1) or not(0) when human control : Mouse_press_l
	BS_Shoot_mode,    // single shoot(1) or continuous shoot(0) when robot control : BigSymbol_ShootMode
	BS_Shoot_command, // simulation mouse pree down(1) or not(0) when robot control : BigSymbol_Mouse_press_l
	Shoot_speed_mode, // high(1) or low(0) speed : Flag_ShootSpeed
	start_fix,        // : fixed_start
	target_angle_fix, // : target_fixed_angle
	PC_online,        // : upperMonitorOnline
	Auto_aim,         // : upperMonitorWork
	BS_recg_state_in, // Is(1) in BS_recg_state or not(0) : Is_Big_Symbol_Recognition_On
	B_has_press_down, // key B press down(1) or not(0) : key_B
	Z_has_press_down, // key Z press down(1) or not(0) : key_Z
	V_has_press_down, // key V press down(1) or not(0) : key_V
	normal_stat,      // first control cycle getting in NORMAL_STATE from other state(0) or not(1) : NormalFlag
	autoaim_stat,     // first control cycle getting in AUTOAIM_STATE from other state(0) or not(1) : UpMonitorFlag
	BS_hit_stat_y,    // first control cycle getting in BS_HIT_STATE from other state(0) or not(1) of yaw: BigSymbolRecgFlag
	BS_hit_stat_p,    // first control cycle getting in BS_HIT_STATE from other state(0) or not(1) of pitch: BigSymbolRecgFlag
	Flag_20,
	Flag_21,
	Flag_22,
	Flag_23,
	Flag_24,
	Flag_25,
	Flag_26,
	Flag_27,
	Flag_28,
	Flag_29,
	Flag_30,
	Flag_31,
	Flag_32,
	Flag_MAX,
}Flag_e;

int Set_Flag(Flag_e Flag_ID);
int Reset_Flag(Flag_e Flag_ID);
int Get_Flag(Flag_e Flag_ID);
void Toggle_Flag(Flag_e Flag_ID);
void Force_Flag(Flag_e Flag_ID, uint8_t init_value);

void Flags_Init(void);

#endif
