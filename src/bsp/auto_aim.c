#include "auto_aim.h"
#include "sys.h"
/***************************自瞄预测函数**************************/
GMAngle_t aimProcess(float yaw,float pit,int16_t *tic){
/*@尹云鹏，自瞄预测及下坠补偿
    参数：绝对角度yaw，pit，计时器地址
    核心思想：
    1.视觉数据需要与真实角度标定，传入参数为目标绝对角度，差分出速度
    2.视觉数据存在误差，以一定时间间隔采样(即高中做实验学过的，高考必考内容)
    3.在循环里速度线性累加与指数衰减对抗
    4.基于pitch的重力下坠补偿
*/
    #define amt 3                //间隔点个数amount，调节amt使时间间隔大约为50ms，即 amt=50ms/1000ms*fps
    static int8_t i, lock;        //计数器，首次进入保护锁
    static float y[amt], p[amt],  //yaw,pit历史
                 tSum, t[amt],    //间隔时间,tic历史
                 wy, wp,          //yaw,pit历史
                 wySum, wpSum;    //角速度累加对抗
    static GMAngle_t in, out;     //上一次值，返回值角度
    
    tSum += *tic - t[i];          //与pid的i计算如出一辙，加上本次并减去amt次以前的时间间隔，得到分频后的间隔
    if(*tic > 150){               //if两次数据时间间隔大于100*2ms，清空历史，进入保护锁
        lock = amt;
        wy = 0; wp = 0;
        in.yaw = yaw;in.pitch = pit;
        out.yaw = yaw;out.pitch = pit;
    }
    in.yaw = (yaw + in.yaw) / 2;       //传入值滤波
    in.pitch = (pit + in.pitch) / 2;
    if(lock){
        lock--;
    }            //函数首次进入保护，只记录数据不预测
    else{
        wy = (wy + (in.yaw - y[i]) / tSum) / 2;    //速度滤波
        wp = (wp + (in.pitch - p[i]) / tSum) / 2;
        wySum += wy;      //角速度累加与指数衰减对抗
        wpSum += wp;
        wySum *= 0.9f;    //指数衰减限制累加,失去物理意义
        wpSum *= 0.9f;
    }
    y[i] = in.yaw;        //yaw历史
    p[i] = in.pitch;      //pit历史
    t[i] = *tic;          //tic历史
    i = (i + 1) % amt;    //amt次之内循环
    out.yaw = (in.yaw + wySum * 20 + out.yaw) / 2;        //实现预测
    out.pitch = (in.pitch + wpSum * 10 + out.pitch) / 2;
//    angle.pit- = 40/angle.pit-0.4;//重力下坠补偿
    *tic = 1;            //时间中断计时器重新开始
    return out;
}
/***************************************************************************************/
