/* second-order kalman filter on stm32 */
#include "kalman_filter.h"
#include "sys.h"
#include "arm_math.h"

#define DELTA_T 0.015
kalman_filter_t kalman_filter_F;

kalman_filter_init_t kalman_filter_init_I = {0,0,0,  /* raw_value filtered_value[2] */
                                            {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, /* xhat_data[4] xhatminus_data[4] K_data[4]*/
                                            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /*Pminus_data[16]*/ 
                                            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /*K_data[16]*/
                                            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /*P_data[16]*/
                                            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /*AT_data[16]*/
                                            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  /*HT_data[16]*/
                                            
                                            {1, 0, DELTA_T,    0   ,
                                             0, 1,    0,    DELTA_T, 
                                             0, 0,    1,       0   , 
                                             0, 0,    0,       1    },  /*A_data[16]*/
                                            
                                            {1, 0, 0, 0,
                                             0, 1, 0, 0,
                                             0, 0, 0, 0,
                                             0, 0, 0, 0},               /*H_data[16]*/
                                            
                                            {1, 20, 0, 0,
                                             20, 1, 0, 0,
                                             0, 0, 1, 20,
                                             0, 0, 20, 1},               /*Q_data[16]*/
                                            
                                            {2000, 0, 0, 0,
                                             0, 2000, 0, 0,
                                             0, 0, 10000, 0,
                                             0, 0, 0, 10000},               /*R_data[16]*/
                                            };

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
    mat_init(&F->z,4,1,(float *)I->z_data);
    mat_init(&F->xhat,4,1,(float *)I->xhat_data);
    mat_init(&F->xhatminus,4,1,(float *)I->xhatminus_data);
    mat_init(&F->K,4,4,(float *)I->K_data);
    mat_init(&F->P,4,4,(float *)I->P_data);
    mat_init(&F->A,4,4,(float *)I->A_data);
    mat_init(&F->H,4,4,(float *)I->H_data);
    mat_init(&F->Q,4,4,(float *)I->Q_data);
    mat_init(&F->R,4,4,(float *)I->R_data);
    mat_init(&F->AT,4,4,(float *)I->AT_data);
    mat_init(&F->Pminus,4,4,(float *)I->Pminus_data);

    mat_trans(&F->H, &F->HT);
}

float *kalman_filter_calc(kalman_filter_t *F, float yaw, float pitch, float v_x, float v_y)
{
  float TEMP_data[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  float TEMP_data21[4] = {0, 0, 0, 0};
  mat TEMP,TEMP21;

  mat_init(&TEMP,4,4,(float *)TEMP_data);
  mat_init(&TEMP21,4,1,(float *)TEMP_data21);

  F->z.pData[0] = yaw;
  F->z.pData[1] = pitch;
  F->z.pData[2] = v_x;
  F->z.pData[3] = v_y;
  
  //1. xhat'(k)= A xhat(k-1)
  mat_mult(&F->A, &F->xhat, &F->xhatminus);

  //2. P'(k) = A P(k-1) AT + Q
  mat_mult(&F->A, &F->P, &F->Pminus);
  mat_mult(&F->Pminus, &F->AT, &TEMP);
  mat_add(&TEMP, &F->Q, &F->Pminus);

  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
  mat_mult(&F->H, &F->Pminus, &F->K);
  mat_mult(&F->K, &F->HT, &TEMP);
  mat_add(&TEMP, &F->R, &F->K);

  mat_inv(&F->K, &F->P);
  mat_mult(&F->Pminus, &F->HT, &TEMP);
  mat_mult(&TEMP, &F->P, &F->K);

  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
  mat_mult(&F->H, &F->xhatminus, &TEMP21);
  mat_sub(&F->z, &TEMP21, &F->xhat);
  mat_mult(&F->K, &F->xhat, &TEMP21);
  mat_add(&F->xhatminus, &TEMP21, &F->xhat);

  //5. P(k) = (1-K(k)H)P'(k)
  mat_mult(&F->K, &F->H, &F->P);
  mat_sub(&F->Q, &F->P, &TEMP);
  mat_mult(&TEMP, &F->Pminus, &F->P);

  F->filtered_value[0] = F->xhat.pData[0];
  F->filtered_value[1] = F->xhat.pData[1];

  return F->filtered_value;
}
///* second-order kalman filter on stm32 */
//#include "kalman_filter.h"
//#include "sys.h"
//#include "arm_math.h"
//#include "math.h"

//#define DELTA_T 0.015
//#define Q1      0.25 * DELTA_T * DELTA_T * DELTA_T * DELTA_T
//#define Q2      0.50 * DELTA_T * DELTA_T * DELTA_T
//#define Q3      0.50 * DELTA_T * DELTA_T * DELTA_T
//#define Q4      1.00 * DELTA_T * DELTA_T

//kalman_filter_t kalman_filter_Fx;
//kalman_filter_t kalman_filter_Fy;

//kalman_filter_init_t kalman_filter_init_Ix = {0,0,0,  /* raw_value filtered_value[2] */
//                                            {0,0}, {0,0}, {0,0}, /* xhat_data[2] xhatminus_data[2] K_data[2]*/
//                                            {0, 0, 0, 0},  /*Pminus_data[4]*/ 
//                                            {0, 0, 0, 0},  /*K_data[4]*/
//                                            {0, 0, 0, 0},  /*P_data[4]*/
//                                            {0, 0, 0, 0},  /*AT_data[4]*/
//                                            {0, 0},  /*HT_data[4]*/
//                                            
//                                            {1, DELTA_T,
//                                             0,   1     },  /*A_data[4]*/
//                                            
//                                            {1, 0},        /*H_data[4]*/
//                                            
//                                            {Q1, Q2, 
//                                             Q3, Q4 },      /*Q_data[4]*/
//                                            
//                                            {2000, 0,
//                                             0, 10000 },        /*R_data[4]*/
//                                            };

//kalman_filter_init_t kalman_filter_init_Iy = {0,0,0,  /* raw_value filtered_value[2] */
//                                            {0,0}, {0,0}, {0,0}, /* xhat_data[2] xhatminus_data[2] K_data[2]*/
//                                            {0, 0, 0, 0},  /*Pminus_data[4]*/ 
//                                            {0, 0, 0, 0},  /*K_data[4]*/
//                                            {0, 0, 0, 0},  /*P_data[4]*/
//                                            {0, 0, 0, 0},  /*AT_data[4]*/
//                                            {0, 0},  /*HT_data[4]*/
//                                            
//                                            {1, DELTA_T,
//                                             0,   1     },  /*A_data[4]*/
//                                            
//                                            {1, 0 },        /*H_data[4]*/
//                                            
//                                            {Q1, Q2, 
//                                             Q3, Q4 },      /*Q_data[4]*/
//                                            
//                                            {2000, 0,
//                                             0, 10000 },        /*R_data[4]*/
//                                            };

//void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
//{
//    mat_init(&F->z,2,1,(float *)I->z_data);
//    mat_init(&F->xhat,2,1,(float *)I->xhat_data);
//    mat_init(&F->xhatminus,2,1,(float *)I->xhatminus_data);
//    mat_init(&F->K,2,2,(float *)I->K_data);
//    mat_init(&F->P,2,2,(float *)I->P_data);
//    mat_init(&F->A,2,2,(float *)I->A_data);
//    mat_init(&F->H,2,1,(float *)I->H_data);
//    mat_init(&F->Q,2,2,(float *)I->Q_data);
//    mat_init(&F->R,2,2,(float *)I->R_data);
//    mat_init(&F->AT,2,2,(float *)I->AT_data);
//    mat_init(&F->Pminus,2,2,(float *)I->Pminus_data);

//    mat_trans(&F->H, &F->HT);
//}

//float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2)
//{
//  float TEMP_data[4] = {0, 0, 0, 0};
//  float TEMP_data21[2] = {0, 0};
//  mat TEMP,TEMP21;

//  mat_init(&TEMP,2,2,(float *)TEMP_data);
//  mat_init(&TEMP21,2,1,(float *)TEMP_data21);

//  F->z.pData[0] = signal1;
//  F->z.pData[1] = signal2;

//  //1. xhat'(k)= A xhat(k-1)
//  mat_mult(&F->A, &F->xhat, &F->xhatminus);

//  //2. P'(k) = A P(k-1) AT + Q
//  mat_mult(&F->A, &F->P, &F->Pminus);
//  mat_mult(&F->Pminus, &F->AT, &TEMP);
//  mat_add(&TEMP, &F->Q, &F->Pminus);

//  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
//  mat_mult(&F->H, &F->Pminus, &F->K);
//  mat_mult(&F->K, &F->HT, &TEMP);
//  mat_add(&TEMP, &F->R, &F->K);

//  mat_inv(&F->K, &F->P);
//  mat_mult(&F->Pminus, &F->HT, &TEMP);
//  mat_mult(&TEMP, &F->P, &F->K);

//  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
//  mat_mult(&F->H, &F->xhatminus, &TEMP21);
//  mat_sub(&F->z, &TEMP21, &F->xhat);
//  mat_mult(&F->K, &F->xhat, &TEMP21);
//  mat_add(&F->xhatminus, &TEMP21, &F->xhat);

//  //5. P(k) = (1-K(k)H)P'(k)
//  mat_mult(&F->K, &F->H, &F->P);
//  mat_sub(&F->Q, &F->P, &TEMP);
//  mat_mult(&TEMP, &F->Pminus, &F->P);

//  F->filtered_value[0] = F->xhat.pData[0];
//  F->filtered_value[1] = F->xhat.pData[1];

//  return F->filtered_value;
//}
