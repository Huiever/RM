#include "mpu9250.h"

imu_t imu = {
			{0,0,0,0,0,0,0,0,0,0},       //raw
			{0,0,0,-2,-15,12,0,0,0},     //offset
			{0,0,0,0,0,0,0,0,0,0,0,0,0}  //rip
			};

u8 MPU_WaitForReady(u8 devaddr);
u8 MPU_Write_Byte(u8 devaddr,u8 reg,u8 data);
u8 MPU_Read_Byte(u8 devaddr,u8 reg);
u8 MPU_Set_Gyro_Fsr(u8 fsr);
u8 MPU_Set_Accel_Fsr(u8 fsr);
u8 MPU_Set_Rate(u16 rate);
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf);

void AK8963_Init(void)
{
  // First extract the factory calibration for each magnetometer axis
	  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  MPU_Write_Byte(AK8963_ADDR, MAG_CNTL1, 0x00); // Power down magnetometer  
  delay_ms(10);
  MPU_Write_Byte(AK8963_ADDR, MAG_CNTL1, 0x0F); // Enter Fuse ROM access mode
  delay_ms(10);
  MPU_Read_Len(AK8963_ADDR, 0x10, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
	//  destination[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
	//  destination[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
	//  destination[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f; 
  MPU_Write_Byte(AK8963_ADDR, MAG_CNTL1, 0x00); // Power down magnetometer  
  delay_ms(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  MPU_Write_Byte(AK8963_ADDR, MAG_CNTL1, 0x01 << 4 | 0x06); // Set magnetometer data resolution and sample ODR
  delay_ms(10);
}
//��ʼ��MPU6500 
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU9250_Init(void){
		IIC_Init();     //��ʼ��IIC����
		if(MPU6500_ID==MPU_Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG)){               //����ID��ȷ
      printf("6500ID is correct!\r\n"); 
			return 0;			
    }else return 1;
		
		uint8_t MPU6500_Init_Data[8][2] = {
		{MPU_PWR_MGMT1_REG,   0X80},      // ��λMPU9250
		{MPU_PWR_MGMT1_REG,   0X00},      // ����MPU9250
		{MPU_INT_EN_REG,      0X00},         // �ر������ж�
		{MPU_USER_CTRL_REG,   0x00},      // I2C��ģʽ�ر�
		{MPU_FIFO_EN_REG,     0X00},    // �ر�FIFO
		{MPU_INTBP_CFG_REG,   0x82},       // INT���ŵ͵�ƽ��Ч������bypassģʽ������ֱ�Ӷ�ȡ������
		{MPU_PWR_MGMT1_REG,   0x01},        // ����CLKSEL,PLL X��Ϊ�ο�
		{MPU_PWR_MGMT2_REG,   0x00},          // ���ٶ��������Ƕ�����
		};

		for(uint8_t index = 0; index < 8; index++){
		MPU_Write_Byte(MPU9250_ADDR,MPU6500_Init_Data[index][0], MPU6500_Init_Data[index][1]);
		delay_ms(1);
		}
		
    MPU_Set_Gyro_Fsr(3);					        	//�����Ǵ�����,��2000dps
	  MPU_Set_Accel_Fsr(0);					       	 	//���ٶȴ�����,��2g
    MPU_Set_Rate(50);						       	 	  //���ò�����50Hz
		
		AK8963_Init();
		
    return 0;	
}

//����MPU9250�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Gyro_Fsr(u8 fsr){
	return MPU_Write_Byte(MPU9250_ADDR,MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ  
}
//����MPU9250���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Accel_Fsr(u8 fsr){
	return MPU_Write_Byte(MPU9250_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}

//����MPU9250�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_LPF(u16 lpf){
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU9250_ADDR,MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}

//����MPU9250�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Rate(u16 rate) {
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU9250_ADDR,MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��¶�ֵ(ԭʼֵ)
void MPU_Get_Temperature(short *temp){
	u8 buf[2]; 
	MPU_Read_Len(MPU9250_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
	*temp=((u8)buf[0]<<8)|buf[1];  
	//*temp=21+((double)raw)/333.87;  
}


//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz){
	u8 buf[6],res; 
	res=MPU_Read_Len(MPU9250_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0){
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
	return res;;
}
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az){
	u8 buf[6],res;  
	res=MPU_Read_Len(MPU9250_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0){
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
	return res;;
}

//�õ�������ֵ(ԭʼֵ)
//mx,my,mz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Magnetometer(short *mx,short *my,short *mz){
	u8 buf[7],res;  
	MPU_Read_Len(AK8963_ADDR,MAG_XOUT_L,7,buf);
	res = buf[6];
	if(!(res&0x08)){
		*mx=((u16)buf[1]<<8)|buf[0];  
		*my=((u16)buf[3]<<8)|buf[2];  
		*mz=((u16)buf[5]<<8)|buf[4];
	}
	//MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11); //AK8963ÿ�ζ����Ժ���Ҫ��������Ϊ���β���ģʽ
	return !res;;
}

void readMagData(short *mx,short *my,short *mz)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if(MPU_Read_Byte(AK8963_ADDR, 0x02) & 0x01) { // wait for magnetometer data ready bit to be set
  MPU_Read_Len(AK8963_ADDR, MAG_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
  uint8_t c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
    *mx= (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
    *my = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) ;  // Data stored as little Endian
    *mz = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) ; 
   }
  }
}

//Get 9 axis data from MPU6500 & AK8963
void IMU_Get_Raw_Data(void)
{
	MPU_Get_Gyroscope(&imu.raw.gx,&imu.raw.gy,&imu.raw.gz);
	MPU_Get_Accelerometer(&imu.raw.ax,&imu.raw.ay,&imu.raw.az);
	MPU_Get_Magnetometer(&imu.raw.mx,&imu.raw.my,&imu.raw.mz);
	MPU_Get_Temperature(&imu.raw.temp);

  imu.raw.gx -= imu.offset.gx;
  imu.raw.gy -= imu.offset.gy;
  imu.raw.gz -= imu.offset.gz;
	
	imu.raw.mx -= imu.offset.mx;
	imu.raw.my -= imu.offset.my;
	imu.raw.mz -= imu.offset.mz;
//	printf("mx:%d my:%d mz:%d\r\n",imu.raw.mx,imu.raw.my,imu.raw.mz);
//	printf("ax:%d ay:%d az:%d\r\n",imu.raw.ax,imu.raw.ay,imu.raw.az);
//	printf("gx:%d gy:%d gz:%d\r\n",imu.raw.gx,imu.raw.gy,imu.raw.gz);
}

//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf){
	u8 i;
	IIC_Start();
	IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
	if(IIC_Wait_Ack()){          //�ȴ�Ӧ��
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg);         //д�Ĵ�����ַ
	IIC_Wait_Ack();             //�ȴ�Ӧ��
	for(i=0;i<len;i++){
		IIC_Send_Byte(buf[i]);  //��������
		if(IIC_Wait_Ack()){      //�ȴ�ACK
			IIC_Stop();
			return 1;
		}
	}
	IIC_Stop();
	return 0;
} 

//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf){ 
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

//IICдһ���ֽ� 
//devaddr:����IIC��ַ
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Byte(u8 addr,u8 reg,u8 data){
	IIC_Start();
	IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
	if(IIC_Wait_Ack()){          //�ȴ�Ӧ��
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg);         //д�Ĵ�����ַ
	IIC_Wait_Ack();             //�ȴ�Ӧ��
	IIC_Send_Byte(data);        //��������
	if(IIC_Wait_Ack()){          //�ȴ�ACK
		IIC_Stop();
		return 1;
	}
	IIC_Stop();
	return 0;
}
//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
u8 MPU_Read_Byte(u8 addr,u8 reg){
	u8 res;
	IIC_Start();
	IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
	IIC_Wait_Ack();             //�ȴ�Ӧ��
	IIC_Send_Byte(reg);         //д�Ĵ�����ַ
	IIC_Wait_Ack();             //�ȴ�Ӧ��
	IIC_Start();                
	IIC_Send_Byte((addr<<1)|1); //����������ַ+������
	IIC_Wait_Ack();             //�ȴ�Ӧ��
	res=IIC_Read_Byte(0);		//������,����nACK  
	IIC_Stop();                 //����һ��ֹͣ����
	return res;  
}

