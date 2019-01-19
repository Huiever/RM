#include "imu.h"
#include "spi.h"
#include "mpu6500_reg.h"
#include "IST8310_reg.h"
#include "delay.h"
#include "usart3.h"
#include "timer.h"
#include <math.h>
#include <string.h>

#include "main.h"

#define BOARD_DOWN (1)
#define IST8310

#define Kp IMU_Kp                                            /* 
                                                              * proportional gain governs rate of 
                                                              * convergence to accelerometer/magnetometer 
																															*/
#define Ki IMU_Ki                                            /* 
                                                              * integral gain governs rate of 
                                                              * convergence of gyroscope biases 
																															*/
volatile float        q0 = 1.0f;
volatile float        q1 = 0.0f;
volatile float        q2 = 0.0f;
volatile float        q3 = 0.0f;
volatile float        exInt, eyInt, ezInt;                   /* error integral */
static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;  
volatile uint32_t     last_update, now_update;               /* Sampling cycle count, ubit ms */
uint8_t               mpu_buff[14];                          /* buffer to save imu raw data */
uint8_t               ist_buff[6];                           /* buffer to save IST8310 raw data */
																									
imu_t								  imu = {
											{0,0,0,0,0,0,0,0,0,0},       //raw
											{0,0,0,0,0,0,
												{-78.53f, -14.46f,30.46f,0.99f,-0.001f, -0.042f,0.988f,-0.054f,1.030f}
											},     //offset
											{0,0,0,0,0,0,0,0,0,0,0,0,0}  //rip
											};

int32_t MPU6500_FIFO[6][11] = {0};//[0]-[9]为最近10次数据 [10]为10次数据的平均值
int16_t IST8310_FIFO[3][11] = {0};//[0]-[9]为最近10次数据 [10]为10次数据的平均值 注：磁传感器的采样频率慢，所以单独列出

uint8_t MPU_id = 0x70;

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//Write a register to MPU6500
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data)
{
  static uint8_t MPU_Tx;//MPU_Rx,
	
  MPU6500_NSS_Low();
  MPU_Tx = reg&0x7f;
  SPI_ReadWriteByte(MPU_Tx);
  MPU_Tx = data;
	SPI_ReadWriteByte(MPU_Tx);
  MPU6500_NSS_High();
  return 0;
}
//Read a register from MPU6500
uint8_t MPU6500_Read_Reg(uint8_t const reg)
{
  static uint8_t MPU_Rx, MPU_Tx; 
  MPU6500_NSS_Low();
  MPU_Tx = reg|0x80;
  SPI_ReadWriteByte(MPU_Tx);
	MPU_Rx=SPI_ReadWriteByte(0x00);//0xff
  MPU6500_NSS_High();
  return MPU_Rx;
}
//Read registers from MPU6500,address begin with regAddr
uint8_t MPU6500_Read_Regs(uint8_t const regAddr, uint8_t *pData, uint8_t len)
{
  static uint8_t  MPU_Tx;//MPU_Rx;//
  MPU6500_NSS_Low();
  MPU_Tx = regAddr|0x80;
	SPI_ReadWriteByte(MPU_Tx);
	for(int i=0;i<len;i++)
  {
		pData[i]=SPI_ReadWriteByte(0x00);//0xff
	}
  MPU6500_NSS_High();
  return 0;
}
//Write IST8310 register through MPU6500
static void IST_Reg_Write_By_MPU(uint8_t addr, uint8_t data)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, addr);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, data);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  delay_ms(10);
}
static uint8_t IST_Reg_Read_By_MPU(uint8_t addr)
{
  uint8_t data;
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_REG, addr);
  delay_ms(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x80);
  delay_ms(10);
  data = MPU6500_Read_Reg(MPU6500_I2C_SLV4_DI);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  delay_ms(10);
  return data;
}
static void MPU_Auto_Read_IST_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, device_address);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
  delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_REG, reg_base_addr);
  delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x03);
  delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
  delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  delay_ms(6);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
  delay_ms(2);
}
uint8_t IST8310_Init(void)
{
  MPU6500_Write_Reg(MPU6500_USER_CTRL, 0x30);
  delay_ms(10);
  MPU6500_Write_Reg(MPU6500_I2C_MST_CTRL, 0x0d);
  delay_ms(10);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
  delay_ms(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
  delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x01);
  delay_ms(10);
  if(IST8310_DEVICE_ID_A != IST_Reg_Read_By_MPU(IST8310_WHO_AM_I))
    return 1; //error
	
	IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x01); 
    delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFA, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFA) != 0x00)
    return 2;
  delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFB) != 0x00)
    return 3;
  delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_AVGCNTL, 0x24);
  if(IST_Reg_Read_By_MPU(IST8310_AVGCNTL) != 0x24)
    return 4;
  delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_PDCNTL, 0xc0);
  if(IST_Reg_Read_By_MPU(IST8310_PDCNTL) != 0xc0)
    return 5;
  delay_ms(10);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  delay_ms(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  delay_ms(10);
  
  MPU_Auto_Read_IST_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
  delay_ms(100);
  return 0;
}

//读取磁力计的数据
void GetIST8310_RawValues(uint8_t* buff)
{
	MPU6500_Read_Regs(MPU6500_EXT_SENS_DATA_00,buff,6);
}


/**********************************************************************************/
/*将MPU6500_ax,MPU6500_ay, MPU6500_az,MPU6500_gx, MPU6500_gy, MPU6500_gz处理后存储*/
/**********************************************************************************/

//[0]-[9]为最近10次数据 [10]为10次数据的平均值
void mpu6500_datasave(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz){

	for(uint8_t i=1;i<10;i++){
		MPU6500_FIFO[0][i-1]=MPU6500_FIFO[0][i];
		MPU6500_FIFO[1][i-1]=MPU6500_FIFO[1][i];
		MPU6500_FIFO[2][i-1]=MPU6500_FIFO[2][i];
		MPU6500_FIFO[3][i-1]=MPU6500_FIFO[3][i];
		MPU6500_FIFO[4][i-1]=MPU6500_FIFO[4][i];
		MPU6500_FIFO[5][i-1]=MPU6500_FIFO[5][i];
	}
	
	MPU6500_FIFO[0][9]=ax;//将新的数据放置到 数据的最后面
	MPU6500_FIFO[1][9]=ay;
	MPU6500_FIFO[2][9]=az;
	MPU6500_FIFO[3][9]=gx;
	MPU6500_FIFO[4][9]=gy;
	MPU6500_FIFO[5][9]=gz;
	
	for(uint8_t j=0;j<6;j++){
			for(uint8_t i=0;i<10;i++){
				MPU6500_FIFO[j][10]+=MPU6500_FIFO[j][i]; //数据量大，int16_t会溢出
			}
			MPU6500_FIFO[j][10]=MPU6500_FIFO[j][10]/10;
	}
}

void IST8310_datasave(int16_t mx,int16_t my,int16_t mz){

	for(uint8_t i=1;i<10;i++){
		IST8310_FIFO[0][i-1]=IST8310_FIFO[0][i];
		IST8310_FIFO[1][i-1]=IST8310_FIFO[1][i];
		IST8310_FIFO[2][i-1]=IST8310_FIFO[2][i];
	}
	IST8310_FIFO[0][9]= mx;//将新的数据放置到 数据的最后面
	IST8310_FIFO[1][9]= my;
	IST8310_FIFO[2][9]= mz;
	
	for(uint8_t j=0;j<3;j++){
		for(uint8_t i=0;i<10;i++){
			IST8310_FIFO[j][10]+=IST8310_FIFO[j][i];
		}
		IST8310_FIFO[j][10]=IST8310_FIFO[j][10]/10;
	}
}


void imu_calibrate(){
	
	float tempx,tempy,tempz;
	tempx=(float)(imu.raw.mx -imu.offset.mag.mx);
	tempy=(float)(imu.raw.my -imu.offset.mag.my);
	tempz=(float)(imu.raw.mz -imu.offset.mag.mz);
	#if ELLIPSOID_FIT
		imu.raw.mx=(int16_t)(imu.offset.mag.b0*tempx+imu.offset.mag.b1*tempy+imu.offset.mag.b2*tempz);
		imu.raw.my=(int16_t)(imu.offset.mag.b1*tempx+imu.offset.mag.b3*tempy+imu.offset.mag.b4*tempz);
		imu.raw.mz=(int16_t)(imu.offset.mag.b2*tempx+imu.offset.mag.b4*tempy+imu.offset.mag.b5*tempz);
	#else
		imu.raw.mx=(int16_t) tempx;
		imu.raw.my=(int16_t) tempy;
		imu.raw.mz=(int16_t) tempz;
	#endif
	imu.rip.mx=(float)(imu.raw.mx/1000);
	imu.rip.my=(float)(imu.raw.my/1000);
	imu.rip.mz=(float)(imu.raw.mz/1000);
	
	imu.raw.ax -= imu.offset.ax;
	imu.raw.ay -= imu.offset.ay;
	imu.raw.az -= imu.offset.az;
	
	imu.raw.gx -= imu.offset.gx;
	imu.raw.gy -= imu.offset.gy;
	imu.raw.gz -= imu.offset.gz;
}

//Get 6 axis data from MPU6500
void IMU_Get_Raw_Data()
{
  MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
	
  imu.raw.ax = mpu_buff[0] << 8 | mpu_buff[1];
  imu.raw.ay = mpu_buff[2] << 8 | mpu_buff[3];
  imu.raw.az = mpu_buff[4] << 8 | mpu_buff[5];
  
  imu.raw.temp = mpu_buff[6] << 8 | mpu_buff[7];
  
  imu.raw.gx = ((mpu_buff[8]  << 8 | mpu_buff[9])  - imu.offset.gx);
  imu.raw.gy = ((mpu_buff[10] << 8 | mpu_buff[11]) - imu.offset.gy);
  imu.raw.gz = ((mpu_buff[12] << 8 | mpu_buff[13]) - imu.offset.gz);
	
	GetIST8310_RawValues(ist_buff);
	memcpy(&imu.raw.mx, ist_buff, 6);\
	
	imu_calibrate();
	mpu6500_datasave(imu.raw.ax,imu.raw.ay,imu.raw.az,imu.raw.gx,imu.raw.gy,imu.raw.gz); 
	IST8310_datasave(imu.raw.mx, imu.raw.my, imu.raw.mz);
	
	imu.raw.ax=(int16_t) MPU6500_FIFO[0][10];
	imu.raw.ay=(int16_t) MPU6500_FIFO[1][10];
	imu.raw.az=(int16_t) MPU6500_FIFO[2][10];
	
	imu.raw.gx=(int16_t) MPU6500_FIFO[3][10];
	imu.raw.gy=(int16_t) MPU6500_FIFO[4][10];
	imu.raw.gz=(int16_t) MPU6500_FIFO[5][10];

	imu.raw.mx=(int16_t) IST8310_FIFO[0][10];
	imu.raw.my=(int16_t) IST8310_FIFO[1][10];
	imu.raw.mz=(int16_t) IST8310_FIFO[2][10];
		
	imu.rip.temp = 21 + imu.raw.temp / 333.87f;
	/* 2000dps -> rad/s */
	imu.rip.gx   = imu.raw.gx / 16.4f / 57.3f; 
  imu.rip.gy   = imu.raw.gy / 16.4f / 57.3f; 
  imu.rip.gz   = imu.raw.gz / 16.4f / 57.3f;
}

//Set the accelerated velocity resolution
uint8_t MPU6500_Set_Accel_Fsr(uint8_t fsr)
{
  return MPU6500_Write_Reg(MPU6500_ACCEL_CONFIG, fsr<<3);
}

//Set the angular velocity resolution
uint8_t MPU6500_Set_Gyro_Fsr(uint8_t fsr)
{
  return MPU6500_Write_Reg(MPU6500_GYRO_CONFIG, fsr<<3);
}

//Initialize the MPU6500
uint8_t MPU6500_Init(void)
{
	MPU6500_NSS_High();
	
	delay_ms(100);
	
	if(MPU_id == MPU6500_Read_Reg(MPU6500_WHO_AM_I)){
//	  printf("MPU6500 init done!");
	}
  uint8_t index = 0;
  uint8_t MPU6500_Init_Data[10][2] = 
  {
    {MPU6500_PWR_MGMT_1,    0x80},      // Reset Device
    {MPU6500_PWR_MGMT_1,    0x03},      // Clock Source - Gyro-Z
    {MPU6500_PWR_MGMT_2,    0x00},      // Enable Acc & Gyro
    {MPU6500_CONFIG,        0x04},      // LPF 98Hz
    {MPU6500_GYRO_CONFIG,   0x18},      // +-2000dps
    {MPU6500_ACCEL_CONFIG,  0x10},      // +-8G
    {MPU6500_ACCEL_CONFIG_2,0x02},      // enable LowPassFilter  Set Acc LPF
    {MPU6500_USER_CTRL,     0x20},      // Enable AUX
  };
  
	for(index = 0; index < 10; index++)
	{
		MPU6500_Write_Reg(MPU6500_Init_Data[index][0], MPU6500_Init_Data[index][1]);
		delay_ms(1);
	}
	
	MPU6500_Set_Gyro_Fsr(3);
	MPU6500_Set_Accel_Fsr(2);
	
	IST8310_Init();
//	printf("%d\r\n", IST8310_Init());
	
	mpu_offset_call();
	init_quaternion();
  return 0;
}

void mpu_offset_call(void)
{
	int i;
	for (i=0; i<300;i++)
	{
		MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

		imu.offset.ax += mpu_buff[0] << 8 | mpu_buff[1];
		imu.offset.ay += mpu_buff[2] << 8 | mpu_buff[3];
		imu.offset.az += mpu_buff[4] << 8 | mpu_buff[5];
	
		imu.offset.gx += mpu_buff[8]  << 8 | mpu_buff[9];
		imu.offset.gy += mpu_buff[10] << 8 | mpu_buff[11];
		imu.offset.gz += mpu_buff[12] << 8 | mpu_buff[13];

		delay_ms(5);
	}
	imu.offset.ax=imu.offset.ax / 300;
	imu.offset.ay=imu.offset.ay / 300;
	imu.offset.az=imu.offset.az / 300;
	imu.offset.gx=imu.offset.gx / 300;
	imu.offset.gy=imu.offset.gx / 300;
	imu.offset.gz=imu.offset.gz / 300;
}

void init_quaternion(void)
{
	int16_t hx, hy;//hz;
	
	hx = imu.raw.mx;
	hy = imu.raw.my;
	//hz = imu.mz;
	
	#ifdef BOARD_DOWN
	if (hx < 0 && hy < 0) 
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = -0.005;
			q1 = -0.199;
			q2 = 0.979;
			q3 = -0.0089;
		}
		else
		{
			q0 = -0.008;
			q1 = -0.555;
			q2 = 0.83;
			q3 = -0.002;
		}
		
	}
	else if (hx < 0 && hy > 0)
	{
		if (fabs(hx / hy)>=1)
		{
			q0 = 0.005;
			q1 = -0.199;
			q2 = -0.978;
			q3 = 0.012;
		}
		else
		{
			q0 = 0.005;
			q1 = -0.553;
			q2 = -0.83;
			q3 = -0.0023;
		}
		
	}
	else if (hx > 0 && hy > 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.0012;
			q1 = -0.978;
			q2 = -0.199;
			q3 = -0.005;
		}
		else
		{
			q0 = 0.0023;
			q1 = -0.83;
			q2 = -0.553;
			q3 = 0.0023;
		}
		
	}
	else if (hx > 0 && hy < 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.0025;
			q1 = 0.978;
			q2 = -0.199;
			q3 = 0.008;			
		}
		else
		{
			q0 = 0.0025;
			q1 = 0.83;
			q2 = -0.56;
			q3 = 0.0045;
		}		
	}
	#else
		if (hx < 0 && hy < 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.195;
			q1 = -0.015;
			q2 = 0.0043;
			q3 = 0.979;
		}
		else
		{
			q0 = 0.555;
			q1 = -0.015;
			q2 = 0.006;
			q3 = 0.829;
		}
		
	}
	else if (hx < 0 && hy > 0)
	{
		if(fabs(hx / hy) >= 1)
		{
			q0 = -0.193;
			q1 = -0.009;
			q2 = -0.006;
			q3 = 0.979;
		}
		else
		{
			q0 = -0.552;
			q1 = -0.0048;
			q2 = -0.0115;
			q3 = 0.8313;
		}
		
	}
	else if (hx > 0 && hy > 0)
	{
		if(fabs(hx / hy) >= 1)
		{
			q0 = -0.9785;
			q1 = 0.008;
			q2 = -0.02;
			q3 = 0.195;
		}
		else
		{
			q0 = -0.9828;
			q1 = 0.002;
			q2 = -0.0167;
			q3 = 0.5557;
		}
		
	}
	else if (hx > 0 && hy < 0)
	{
		if(fabs(hx / hy) >= 1)
		{
			q0 = -0.979;
			q1 = 0.0116;
			q2 = -0.0167;
			q3 = -0.195;			
		}
		else
		{
			q0 = -0.83;
			q1 = 0.014;
			q2 = -0.012;
			q3 = -0.556;
		}		
	}
	#endif
}

void IMU_AHRSupdate(void)
{
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez, halfT;
	float tempq0,tempq1,tempq2,tempq3;

	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;   
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;   

	gx = imu.rip.gx;
	gy = imu.rip.gy;
	gz = imu.rip.gz;
	ax = imu.raw.ax;
	ay = imu.raw.ay;
	az = imu.raw.az;
	mx = imu.raw.mx;
	my = imu.raw.my;
	mz = imu.raw.mz;

	now_update  = Get_Time_Micros(); //ms
	halfT       = ((float)(now_update - last_update) / 2000.0f);
	last_update = now_update;
	
	/* Fast inverse square-root */
	norm = invSqrt(ax*ax + ay*ay + az*az);       
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	
	#ifdef IST8310
		norm = invSqrt(mx*mx + my*my + mz*mz);          
		mx = mx * norm;
		my = my * norm;
		mz = mz * norm; 
	#else
		mx = 0;
		my = 0;
		mz = 0;		
	#endif
	/* compute reference direction of flux */
	hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
	hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
	hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
	bx = sqrt((hx*hx) + (hy*hy));
	bz = hz; 
	
	/* estimated direction of gravity and flux (v and w) */
	vx = 2.0f*(q1q3 - q0q2);
	vy = 2.0f*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
	wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
	wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
	
	/* 
	 * error is sum of cross product between reference direction 
	 * of fields and direction measured by sensors 
	 */
	ex = (ay*vz - az*vy) + (my*wz - mz*wy);
	ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
	ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

	/* PI */
	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;	
		ezInt = ezInt + ez * Ki * halfT;
		
		gx = gx + Kp*ex + exInt;
		gy = gy + Kp*ey + eyInt;
		gz = gz + Kp*ez + ezInt;
	}
	
	tempq0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
	tempq1 = q1 + (q0*gx + q2*gz - q3*gy) * halfT;
	tempq2 = q2 + (q0*gy - q1*gz + q3*gx) * halfT;
	tempq3 = q3 + (q0*gz + q1*gy - q2*gx) * halfT;  

	/* normalise quaternion */
	norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
	q0 = tempq0 * norm;
	q1 = tempq1 * norm;
	q2 = tempq2 * norm;
	q3 = tempq3 * norm;
}

/**
	* @brief  update imu attitude
  * @param  
	* @retval 
  * @usage  call in main() function
	*/

volatile  float yaw_temp,pitch_temp,roll_temp;
volatile  float last_yaw_temp,last_pitch_temp,last_roll_temp;
static int yaw_count = 0;
void IMU_getYawPitchRoll(void)
{
	// yaw    -pi----pi
	imu.rip.yaw = -atan2(2 * q1 * q2 + 2 * q0* q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 57.3; 
	// pitch  -pi/2--- pi/2 
	imu.rip.pit = -asin(-2 * q1 * q3 + 2 * q0 * q2)* 57.3; 
	// roll   -pi-----pi
	imu.rip.rol = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1)* 57.3; 

	last_yaw_temp = yaw_temp;
	yaw_temp = imu.rip.yaw; 
	if(yaw_temp-last_yaw_temp>=330){  //yaw轴角度经过处理后变成连续的
		yaw_count--;
	}
	else if (yaw_temp-last_yaw_temp<=-330){
		yaw_count++;
	}
	imu.rip.yaw = yaw_temp + yaw_count*360;  //yaw轴角度
}

float get_yaw_angle(void){
  return imu.rip.yaw;
}

float get_imu_wx(void){
  return imu.rip.gx;
}

float get_imu_wy(void){
  return imu.rip.gy;
}

float get_imu_wz(void){
  return imu.rip.gz;
}

int16_t get_mpu_gx(void){
  return imu.raw.gx;
}

int16_t get_mpu_gy(void){
  return imu.raw.gy;
}

int16_t get_mpu_gz(void){
  return imu.raw.gz;
}

void imu_main(void){
  IMU_Get_Raw_Data();
	IMU_AHRSupdate();
	IMU_getYawPitchRoll();
	delay_ms(5);
#if Monitor_IMU_Angle==1
	printf("yaw_angle:%8.3lf  pit_angle:%8.3lf  rol_angle:%8.3lf\r\n", imu.rip.yaw, imu.rip.pit, imu.rip.rol);
	delay_ms(5);
#endif
#if Monitor_IMU_Accel==1
	printf("wx:%8.3lf  wy:%8.3lf  wz:%8.3lf\r\n", imu.rip.gx, imu.rip.gy, imu.rip.gz);
	delay_ms(5);
#endif
#if Monitor_IMU_Mag==1
	printf("%d,%d,%d\r\n",imu.raw.mx,imu.raw.my,imu.raw.mz);
  delay_ms(5);
#endif
}
