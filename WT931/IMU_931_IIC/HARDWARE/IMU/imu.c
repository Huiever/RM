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

//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 IMU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf){ 
	IIC_Start();
	IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
	if(IIC_Wait_Ack()){          //�ȴ�Ӧ��
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg);         //д�Ĵ�����ַ
	IIC_Wait_Ack();             //�ȴ�Ӧ��
	IIC_Start();                
	IIC_Send_Byte((addr<<1)|1); //����������ַ+������
	IIC_Wait_Ack();             //�ȴ�Ӧ��
	while(len){
		if(len==1)
			*buf=IIC_Read_Byte(0);//������,����nACK 
		else
			*buf=IIC_Read_Byte(1);		//������,����ACK  
		len--;
		buf++;  
	}
	IIC_Stop();                 //����һ��ֹͣ����
	return 0;       
}
