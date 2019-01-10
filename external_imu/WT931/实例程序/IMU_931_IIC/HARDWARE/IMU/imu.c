#include "imu.h"

void ShortToChar(short sData,unsigned char cData[]);
short CharToShort(unsigned char cData[]);
u8 IMU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf);

void get_yaw_picth_roll_angle(float *pitch,float *roll,float *yaw)
{
	unsigned char chrTemp[10];
	IMU_Read_Len(0x50, ROLL_REG, 6,&chrTemp[0]);
	*roll  = (float)CharToShort(&chrTemp[0])/32768*180;
	*pitch = (float)CharToShort(&chrTemp[2])/32768*180;
	*yaw   = (float)CharToShort(&chrTemp[4])/32768*180;	
}

void ShortToChar(short sData,unsigned char cData[])
{
	cData[0]=sData&0xff;
	cData[1]=sData>>8;
}
short CharToShort(unsigned char cData[])
{
	return ((short)cData[1]<<8)|cData[0];
}

//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 IMU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf){ 
	IIC_Start();
	IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
	if(IIC_Wait_Ack()){          //等待应答
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg);         //写寄存器地址
	IIC_Wait_Ack();             //等待应答
	IIC_Start();                
	IIC_Send_Byte((addr<<1)|1); //发送器件地址+读命令
	IIC_Wait_Ack();             //等待应答
	while(len){
		if(len==1)
			*buf=IIC_Read_Byte(0);//读数据,发送nACK 
		else
			*buf=IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++;  
	}
	IIC_Stop();                 //产生一个停止条件
	return 0;       
}
