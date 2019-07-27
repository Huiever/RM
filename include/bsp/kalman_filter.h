#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_
#include "arm_math.h"

#define mat         arm_matrix_instance_f32 
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32


typedef struct
{
  float raw_value;
  float filtered_value[2];
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter_t;

typedef struct
{
  float raw_value;
  float filtered_value[2];
  float xhat_data[4], xhatminus_data[4], z_data[4],Pminus_data[16], K_data[16];
  float P_data[16];
  float AT_data[16], HT_data[16];
  float A_data[16];
  float H_data[16];
  float Q_data[16];
  float R_data[16];
} kalman_filter_init_t;

extern kalman_filter_t kalman_filter_F;
extern kalman_filter_init_t kalman_filter_init_I;
extern void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
extern float *kalman_filter_calc(kalman_filter_t *F, float yaw, float pitch, float v_x, float v_y);

#endif
//#ifndef _KALMAN_FILTER_H_
//#define _KALMAN_FILTER_H_
//#include "arm_math.h"

//#define mat         arm_matrix_instance_f32 
//#define mat_init    arm_mat_init_f32
//#define mat_add     arm_mat_add_f32
//#define mat_sub     arm_mat_sub_f32
//#define mat_mult    arm_mat_mult_f32
//#define mat_trans   arm_mat_trans_f32
//#define mat_inv     arm_mat_inverse_f32


//typedef struct
//{
//  float raw_value;
//  float filtered_value[2];
//  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
//} kalman_filter_t;

//typedef struct
//{
//  float raw_value;
//  float filtered_value[2];
//  float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
//  float P_data[4];
//  float AT_data[4], HT_data[2];
//  float A_data[4];
//  float H_data[2];
//  float Q_data[4];
//  float R_data[4];
//} kalman_filter_init_t;

//extern kalman_filter_t kalman_filter_Fx;
//extern kalman_filter_t kalman_filter_Fy;
//extern kalman_filter_init_t kalman_filter_init_Ix;
//extern kalman_filter_init_t kalman_filter_init_Iy;
//extern void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
//extern float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2);

//#endif
