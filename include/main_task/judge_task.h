#ifndef _JUDGE_H
#define _JUDGE_H

#include "sys.h"
#include "stdio.h"
#include "delay.h"
#include "stm32f4xx_conf.h"
#include "usart3.h"
#include "string.h"

typedef struct{
    u8     SOF;            //数据帧起始字节，固定值为0xA5;
    u16 DataLength;        //数据帧内Data长度;
    u8    Seq;            //包序号;
    u8    CRC8;            //帧头CRC8;
}FrameHeader_t;

typedef enum {
    CmdID_1        =    0x0001,            //比赛机器人状态，10Hz频率周期发送;
    CmdID_2        =    0x0002,            //伤害数据，收到伤害时发送;
    CmdID_3        =    0x0003,            //实时射击数据，发射弹丸时发送;
    CmdID_4        =    0x0004,            //实时功率和热量数据，50Hz频率周期发送;
    CmdID_5        =    0x0005,            //实时场地交互数据，50Hz频率周期发送;
    CmdID_6        =    0x0006,            //比赛结果数据，比赛结束时发送;
    CmdID_7        =    0x0007,            //获取到buff,激活机关后发送一次;
    CmdID_8        =    0x0100             //参赛队自定义数据，用于显示在操作界面;
}CmdID_t;

typedef  __packed struct  {
  uint16_t remainTime;              //比赛剩余时间
  uint16_t remainLifeValue;          //剩余血量
    uint8_t CurrentLevel;            //当前等级
    uint16_t MaxLifeValue;            //最大血量
}tStudentGameProcessInfo;  

typedef  __packed struct  {
  uint8_t strikeId:4;  
  uint8_t type:4;                  //血量变化类型
  uint16_t data;                  //血量变化值
}tLifeChangedInfo;  

typedef __packed struct  {
    //new
    uint8_t bulletType;                //子弹类型
    uint8_t bulletFreq;                //射频
    float    bulletSpeed;            //射速
    //old
//    float realBulletShootSpeed;      //子弹射速
//    float realBulletShootFreq;      //子弹射频
//    float realGolfShootSpeed;          //高尔夫子弹射速
//    float realGolfShootFreq;          //高尔夫子弹射频
}tStudentInterRealShootData;  

typedef struct {
    u8     validFlage;                    //位置、角度信息有效标志位-----0:无效;1:有效;
    u32    x;                            //位置X坐标值;
    u32    y;                            //位置Y坐标值;
    u32    z;                            //位置Z坐标值;
    u32    yaw;                        //枪口朝向角度值;
}position_t;

typedef struct{
    u16 stageRemianTime;            //当前阶段剩余时间，单位s;
    u8    gameProgress;                //当前比赛阶段;
                                    //0:未开始比赛;
                                    //1:准备阶段;
                                    //2:自检阶段;
                                    //3:5s倒计时;
                                    //4:对战中;
                                    //5:比赛结算中;
    u8    robotLevel;                    //机器人当前等级;
    u16    remainHP;                    //机器人当前血量;
    u16    maxHP;                        //机器人满血量;
    position_t    position;            //位置，角度信息;
}extGameRobotState_t;

typedef struct{
    u8    armorType:    4;            //数据的0-3bits:若变化类型为装甲伤害，标识装甲ID；
                                                            //0x0:0号装甲（前 ）;
                                                            //0x1:1号装甲（左 ）;
                                                            //0x2:2号装甲（后 ）;
                                                            //0x3:3号装甲（右 ）;
                                                            //0x4:4号装甲（上1）;
                                                            //0x5:5号装甲（上2）;
                                                            //其他保留;
    u8    hurtType:    4;            //数据的4-7bits:血量变化类型；
                                                            //0x0:装甲伤害（受到攻击）;
                                                            //0x1:模块掉线;
                                                            //0x2:弹丸超速;
                                                            //0x3:弹丸超频;
                                                            //0x4:枪口超热量;
                                                            //0x5:底盘超功率;
}extRobotHurt_t;

typedef struct{
    u8    bulletType;                //弹丸类型;
                                //0:17mm弹丸;
                                //1:42mm弹丸;
    u8    bulletFreq;                //弹丸射频;
    u32    bulletSpeed;            //弹丸射速;
    u32    reseved;                //保留;
}extShootData_t;

typedef struct{
    float    chassisVolt;        //底盘输出电压;
    float    chassisCurrent;        //底盘输出电流;
    float    chassisPower;        //底盘输出功率;
    float    chassisPowerBuffer;    //底盘功率缓冲;
    uint16_t    shooterHeat17;    //17mm枪口热量;
    uint16_t    shooterHeat42;    //42mm枪口热量;
}extPowerHeatData_t;

typedef struct{
    u8    cardType;                //卡类型;
                                //0:攻击加成卡;
                                //1:防御加成卡;
                                //2:红方加血卡;
                                //3:蓝方加血卡;
                                //4:红方大能量机关卡;
                                //5:蓝方大能量机关卡;
    u8    cardIdx;                //卡索引号，可用于区分不同区域
}extRfidDetect_t;

typedef struct{
    u8    winner;                    //比赛结果;
                                //0:平局;
                                //1:红方胜;
                                //2:蓝方胜;
}extGameResult_t;

typedef struct{
    u8    buffType;                //Buff类型;
                                //0:攻击加成;
                                //1:防御加成;
                                //2:获得大能量机关;
    u8    buffAddition;            //加成百分比（比如10代表10%）
}extGetBuff_t;

typedef struct{
    float    data1;                    //自定义数据1;
    float    data2;                    //自定义数据2;
    float    data3;                    //自定义数据3;
    uint8_t    mask;                    //自定义数据4;
}extShowData_t;

typedef struct{
    FrameHeader_t FrameHeader;
    CmdID_t    CmdID;
    union{
        extGameRobotState_t    extGameRobotState;            //0x0001:比赛机器人状态;
        extRobotHurt_t        extRobotHurt;                //0x0002:伤害数据;
        extShootData_t        extShootData;                //0x0003:实时射击信息;
        extPowerHeatData_t    extPowerHeatData;            //0x0004:实时功率热量数据;
        extRfidDetect_t        extRfidDetect;                //0x0005:场地交互数据;
        extGameResult_t        extGameResult;                //0x0006:比赛胜负数据;
        extGetBuff_t        extGetBuff;                    //0x0007:Buff获取数据;
        extShowData_t        extShowData;                //0x0008:参赛队自定义数据;
    }Data;
    u16    FrameTail;
}DMA1_Stream1_buffer_t;

extern tStudentGameProcessInfo GameProcessInfo;
extern tLifeChangedInfo LifeChangedInfo;
extern tStudentInterRealShootData RealShootData;
extern extPowerHeatData_t PowerHeatData;
extern DMA1_Stream1_buffer_t DMA1_Stream1_buffer;

u8 Get_CRC8_Check_Sum( u8 *pchMessage, u32 dwLength, u8 ucCRC8 );
u16 Verify_CRC8_Check_Sum( u8 *pchMessage, u16 dwLength );
void Append_CRC8_Check_Sum( u8 *pchMessage, u32 dwLength );
u16 Get_CRC16_Check_Sum( u8 *pchMessage, u32 dwLength, uint16_t wCRC );
u32 Verify_CRC16_Check_Sum( u8 *pchMessage, u32 dwLength );
void Append_CRC16_Check_Sum( u8 * pchMessage, u32 dwLength );

float storage2float(uint8_t *storage);
void Judge_Init(void);
void USART2_IRQHandler(void);
void DMA1_Stream5_IRQHandler(void);

uint16_t JUDGE_GET_Sentry_HeatData( void );
uint8_t JUDGE_GET_Sentry_LevelData( void );

/** 
  * @brief  frame header structure definition
  */
//typedef __packed struct
//{
//  uint8_t  sof;
//  uint16_t data_length;
//  uint8_t  seq;
//  uint8_t  crc8;
//} frame_header_t;

#define COMPUTER_FRAME_BUFLEN 5
#define UP_REG_ID 0xA5
#define HEADER_LEN 5
#define CMD_LEN 2
#define CRC_LEN 2

void data_pack_handle(uint8_t *p_data, uint16_t len);
void TIM3_Configuration(void);

void Set_Judge_Send_Data( uint8_t i, float data);
void Set_Judge_Send_Flag( uint8_t i, uint8_t data );

#endif




