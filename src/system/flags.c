#include "flags.h"
#include "beep.h"
#include <stdio.h>
#include <inttypes.h>
 
/***********JSP ©copyright***********/
/*******Version 1.0 2018.07.26*******/

static volatile uint32_t FLAGS = 0x00000000;
static volatile uint8_t  FLAGS_COUNT = 0;
 
void Reset_All_Flags(void){
    FLAGS &= 0x00000000;
}

void Set_All_Flags(void){
    FLAGS |= 0xFFFFFFFF;
}

int Creat_Flag(Flag_e Flag_ID, uint32_t init_value){
    if (FLAGS_COUNT >= 32){
        printf("All the flags has been used out!\r\n");
        Sing_bad_case();
        return 0;
    }
    else if (Flag_ID >= Flag_MAX){
        printf("The flag ID is too big!\r\n");
        Sing_bad_case();
        return 0;
    }
    else if (init_value != 0 && init_value != 1){
        printf("Initial flag value is not valid!\r\n");
        Sing_bad_case();
        return 0;
    }
    else{
        FLAGS |= ((0x00000001 & init_value) << Flag_ID);
        FLAGS_COUNT++;
        return 1;
    }
}

int Set_Flag(Flag_e Flag_ID){
    if (Flag_ID >= Flag_MAX){
        printf("The flag ID is too big!\r\n");
        Sing_bad_case();
        return 0;
    }
    else{
        FLAGS |= (0x00000001 << Flag_ID);
        return 1;
    }
}

int Reset_Flag(Flag_e Flag_ID){
    if (Flag_ID >= Flag_MAX){
        printf("The flag ID is too big!\r\n");
        Sing_bad_case();
        return 0;
    }
    else{
        FLAGS &= ~(0x00000001 << Flag_ID);
        return 1;
    }
}

int Get_Flag(Flag_e Flag_ID){
    if (Flag_ID >= Flag_MAX){
        printf("The flag ID is too big!\r\n");
        Sing_bad_case();
        return 0;
    }
    else if (((FLAGS >> Flag_ID) & 0x00000001)){
        return 1;
    }
    else{
        return 0;
    }
}

void Toggle_Flag(Flag_e Flag_ID){
  if (Flag_ID >= Flag_MAX){
        printf("The flag ID is too big!\r\n");
        Sing_bad_case();
    }
    else{
        FLAGS = FLAGS ^ (0x00000001 << Flag_ID);
    }
}

void Force_Flag(Flag_e Flag_ID, uint8_t init_value){
  if (Flag_ID >= Flag_MAX){
        printf("The flag ID is too big!\r\n");
        Sing_bad_case();
    }
    else if (init_value != 0 && init_value != 1){
        printf("Initial flag value is not valid!\r\n");
        Sing_bad_case();
    }
    else if(init_value == 0){
      FLAGS &= ~(0x00000001 << Flag_ID);
    }
    else{
      FLAGS |= (0x00000001 << Flag_ID);
    }
}

/* You can and //only// can create_flag() here! */
void Flags_Init(void){
    Creat_Flag(Shoot, 0);
    Creat_Flag(Shoot_mode, 0);
    Creat_Flag(Fric_accel_ok, 0);
    Creat_Flag(Shoot_command, 0);
    Creat_Flag(BS_Shoot_mode, 0);
    Creat_Flag(BS_Shoot_command, 0);
    Creat_Flag(Shoot_speed_mode, 0);
    Creat_Flag(start_fix, 0);
    Creat_Flag(target_angle_fix, 0);
    Creat_Flag(PC_ack, 0);
    Creat_Flag(Auto_aim_debug, 0);
    Creat_Flag(Gimble_ok, 0);
    Creat_Flag(Camera_error, 0);
    Creat_Flag(Z_has_press_down, 0);
    Creat_Flag(V_has_press_down, 0);
    Creat_Flag(normal_stat, 0);
    Creat_Flag(autoaim_stat, 0);
    Creat_Flag(BS_hit_stat_y, 0);
    Creat_Flag(BS_hit_stat_p, 0);
}
