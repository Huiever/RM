#include "mpu9250.h"

imu_t imu = {
			{0,0,0,0,0,0,0,0,0,0},       //raw
			{0,0,0,-2,-15,12,0,0,0},     //offset
			{0,0,0,0,0,0,0,0,0,0,0,0,0}  //rip
			};

volatile float q0 = 1.0f;
volatile float q1 = 0.0f;
volatile float q2 = 0.0f;
volatile float q3 = 0.0f;

int16_t MPU6050_FIFO[6][11] = {0};//[0]-[9]为最近10次数据 [10]为10次数据的平均值
int16_t AK8963_FIFO[3][11] = {0};//[0]-[9]为最近10次数据 [10]为10次数据的平均值 注：磁传感器的采样频率慢，所以单独列出
			
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
//初始化MPU6500 
//返回值:0,成功
//    其他,错误代码
u8 MPU9250_Init(void){
		IIC_Init();     //初始化IIC总线
		if(MPU6500_ID==MPU_Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG)){               //器件ID正确
      printf("6500ID is correct!\r\n"); 
			return 0;			
    }else return 1;
		
		uint8_t MPU6500_Init_Data[8][2] = {
		{MPU_PWR_MGMT1_REG,   0X80},      // 复位MPU9250
		{MPU_PWR_MGMT1_REG,   0X00},      // 唤醒MPU9250
		{MPU_INT_EN_REG,      0X00},         // 关闭所有中断
		{MPU_USER_CTRL_REG,   0x00},      // I2C主模式关闭
		{MPU_FIFO_EN_REG,     0X00},    // 关闭FIFO
		{MPU_INTBP_CFG_REG,   0x82},       // INT引脚低电平有效，开启bypass模式，可以直接读取磁力计
		{MPU_PWR_MGMT1_REG,   0x01},        // 设置CLKSEL,PLL X轴为参考
		{MPU_PWR_MGMT2_REG,   0x00},          // 加速度与陀螺仪都工作
		};

		for(uint8_t index = 0; index < 8; index++){
		MPU_Write_Byte(MPU9250_ADDR,MPU6500_Init_Data[index][0], MPU6500_Init_Data[index][1]);
		delay_ms(1);
		}
		
    MPU_Set_Gyro_Fsr(3);					        	//陀螺仪传感器,±2000dps
	  MPU_Set_Accel_Fsr(0);					       	 	//加速度传感器,±2g
    MPU_Set_Rate(50);						       	 	  //设置采样率50Hz
		
		AK8963_Init();
		
    return 0;	
}

//设置MPU9250陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Gyro_Fsr(u8 fsr){
	return MPU_Write_Byte(MPU9250_ADDR,MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}
//设置MPU9250加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Accel_Fsr(u8 fsr){
	return MPU_Write_Byte(MPU9250_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}

//设置MPU9250的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_LPF(u16 lpf){
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU9250_ADDR,MPU_CFG_REG,data);//设置数字低通滤波器  
}

//设置MPU9250的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Rate(u16 rate) {
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU9250_ADDR,MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

//得到温度值(原始值)
void MPU_Get_Temperature(short *temp){
	u8 buf[2]; 
	MPU_Read_Len(MPU9250_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
	*temp=((u8)buf[0]<<8)|buf[1];  
	//*temp=21+((double)raw)/333.87;  
}


//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
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
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
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

//得到磁力计值(原始值)
//mx,my,mz:磁力计x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Magnetometer(short *mx,short *my,short *mz){
	u8 buf[7],res;  
	MPU_Read_Len(AK8963_ADDR,MAG_XOUT_L,7,buf);
	res = buf[6];
	if(!(res&0x08)){
		*mx=((u16)buf[1]<<8)|buf[0];  
		*my=((u16)buf[3]<<8)|buf[2];  
		*mz=((u16)buf[5]<<8)|buf[4];
	}
	//MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11); //AK8963每次读完以后都需要重新设置为单次测量模式
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

//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf){
	u8 i;
	IIC_Start();
	IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
	if(IIC_Wait_Ack()){          //等待应答
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg);         //写寄存器地址
	IIC_Wait_Ack();             //等待应答
	for(i=0;i<len;i++){
		IIC_Send_Byte(buf[i]);  //发送数据
		if(IIC_Wait_Ack()){      //等待ACK
			IIC_Stop();
			return 1;
		}
	}
	IIC_Stop();
	return 0;
} 

//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf){ 
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

//IIC写一个字节 
//devaddr:器件IIC地址
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Byte(u8 addr,u8 reg,u8 data){
	IIC_Start();
	IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
	if(IIC_Wait_Ack()){          //等待应答
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg);         //写寄存器地址
	IIC_Wait_Ack();             //等待应答
	IIC_Send_Byte(data);        //发送数据
	if(IIC_Wait_Ack()){          //等待ACK
		IIC_Stop();
		return 1;
	}
	IIC_Stop();
	return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 MPU_Read_Byte(u8 addr,u8 reg){
	u8 res;
	IIC_Start();
	IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
	IIC_Wait_Ack();             //等待应答
	IIC_Send_Byte(reg);         //写寄存器地址
	IIC_Wait_Ack();             //等待应答
	IIC_Start();                
	IIC_Send_Byte((addr<<1)|1); //发送器件地址+读命令
	IIC_Wait_Ack();             //等待应答
	res=IIC_Read_Byte(0);		//读数据,发送nACK  
	IIC_Stop();                 //产生一个停止条件
	return res;  
}

// Fast inverse square-root
/**************************实现函数********************************************
*函数原型:	   float invSqrt(float x)
*功　　能:	   快速计算 1/Sqrt(x) 	
输入参数： 要计算的值
输出参数： 结果
*******************************************************************************/
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**********************************************************************************/
/*将MPU6050_ax,MPU6050_ay, MPU6050_az,MPU6050_gx, MPU6050_gy, MPU6050_gz处理后存储*/
/**********************************************************************************/

void MPU6500_DataSave(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz){ //[0]-[9]为最近10次数据 [10]为10次数据的平均值

	uint8_t i = 0;
	int32_t sum=0;
	
	for(i=1;i<10;i++){
		MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
		MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
		MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
		MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
		MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
		MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
	}
	
	MPU6050_FIFO[0][9]=ax;//将新的数据放置到 数据的最后面
	MPU6050_FIFO[1][9]=ay;
	MPU6050_FIFO[2][9]=az;
	MPU6050_FIFO[3][9]=gx;
	MPU6050_FIFO[4][9]=gy;
	MPU6050_FIFO[5][9]=gz;
	
	for(i=0;i<10;i++){     //求当前数组的合，再取平均值	
		 sum+=MPU6050_FIFO[0][i];
	}
	MPU6050_FIFO[0][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[1][i];
	}
	MPU6050_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[2][i];
	}
	MPU6050_FIFO[2][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[3][i];
	}
	MPU6050_FIFO[3][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[4][i];
	}
	MPU6050_FIFO[4][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[5][i];
	}
	MPU6050_FIFO[5][10]=sum/10;
	
}

void AK8963_DataSave(int16_t x,int16_t y,int16_t z)
{
	uint8_t i = 0;
	int32_t sum=0;

	for(i=1;i<10;i++){
		AK8963_FIFO[0][i-1]=AK8963_FIFO[0][i];
		AK8963_FIFO[1][i-1]=AK8963_FIFO[1][i];
		AK8963_FIFO[2][i-1]=AK8963_FIFO[2][i];
	}
	AK8963_FIFO[0][9]= x;//将新的数据放置到 数据的最后面
	AK8963_FIFO[1][9]= y;
	AK8963_FIFO[2][9]= z;
	
	for(i=0;i<10;i++)//求当前数组的合，再取平均值
	{	
		 sum+=AK8963_FIFO[0][i];
	}
	AK8963_FIFO[0][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=AK8963_FIFO[1][i];
	}
	AK8963_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=AK8963_FIFO[2][i];
	}
	AK8963_FIFO[2][10]=sum/10;
//		if(MagMaxMinData.MinMagX>HMC5883_FIFO[0][10])
//	{
//		MagMaxMinData.MinMagX=(int16_t)HMC5883_FIFO[0][10];
//	}
//	if(MagMaxMinData.MinMagY>HMC5883_FIFO[1][10])
//	{
//		MagMaxMinData.MinMagY=(int16_t)HMC5883_FIFO[1][10];
//	}
//	if(MagMaxMinData.MinMagZ>HMC5883_FIFO[2][10])
//	{
//		MagMaxMinData.MinMagZ=(int16_t)HMC5883_FIFO[2][10];
//	}

//	if(MagMaxMinData.MaxMagX<HMC5883_FIFO[0][10])
//	{
//		MagMaxMinData.MaxMagX=(int16_t)HMC5883_FIFO[0][10];		
//	}
//	if(MagMaxMinData.MaxMagY<HMC5883_FIFO[1][10])
//	{
//		MagMaxMinData.MaxMagY = HMC5883_FIFO[1][10];
//	}
//	if(MagMaxMinData.MaxMagZ<HMC5883_FIFO[2][10])
//	{
//		MagMaxMinData.MaxMagZ=(int16_t)HMC5883_FIFO[2][10];
//	}		
//	}
}

void AK8963_getlastValues(int16_t *x,int16_t *y,int16_t *z) 
{
    *x = AK8963_FIFO[0][10];
    *y = AK8963_FIFO[1][10]; 
    *z = AK8963_FIFO[2][10]; 
}


/**************************实现函数********************************************
*函数原型:	   void Init_Quaternion
*功　　能:	 初始化四元数
输入参数： 当前的测量值。
输出参数：没有
*******************************************************************************/
//初始化IMU数据
#define BOARD_DOWN 1   //板子正面朝下摆放

void Init_Quaternion()//根据测量数据，初始化q0,q1,q2.q3，从而加快收敛速度
{
	int16_t hx,hy,hz;
	AK8963_getlastValues(&hx,&hy,&hz);
	#ifdef BOARD_DOWN
	if(hx<0 && hy <0)   //OK
	{
		if(fabs(hx/hy)>=1)
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
	else if (hx<0 && hy > 0) //OK
	{
		if(fabs(hx/hy)>=1)   
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
	else if (hx > 0 && hy > 0)   //OK
	{
		if(fabs(hx/hy)>=1)
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
	else if (hx > 0 && hy < 0)     //OK
	{
		if(fabs(hx/hy)>=1)
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
		if(hx<0 && hy <0)
	{
		if(fabs(hx/hy)>=1)
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
	else if (hx<0 && hy > 0)
	{
		if(fabs(hx/hy)>=1)
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
	else if (hx>0 && hy > 0)
	{
		if(fabs(hx/hy)>=1)
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
		if(fabs(hx/hy)>=1)
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
	
	//根据hx hy hz来判断q的值，取四个相近的值做逼近即可,初始值可以由欧拉角转换到四元数计算得到
	 
}

/**************************实现函数********************************************
*函数原型:	   void IMU_AHRSupdate
*功　　能:	 更新AHRS 更新四元数 
输入参数： 当前的测量值。
输出参数：没有
*******************************************************************************/
#define Kp 10.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.01f   // integral gain governs rate of convergence of gyroscope biases

void IMU_AHRSupdate(void) {

		volatile static float exInt, eyInt, ezInt;  // 误差积分
		volatile static float gx, gy, gz, ax, ay, az, mx, my, mz;   //作用域仅在此文件中
		volatile static uint32_t lastUpdate, now; // 采样周期计数 单位 us
	
    float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez,halfT;
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
	
	  ax = imu.raw.ax;
    ay = imu.raw.ay;
    az = imu.raw.az;
    gx = imu.rip.gx;
    gy = imu.rip.gy;
    gz = imu.rip.gz;
    mx = imu.raw.mx;
    my = imu.raw.my;
    mz = imu.raw.mz;	
	

		now  = Get_Time_Micros(); //ms
		halfT  = ((float)(now - lastUpdate) / 2000.0f);
		lastUpdate = now;


//    now = Get_Time_Micros();  //读取时间 单位是us   
//    if(now<lastUpdate)
//    {
//    //halfT =  ((float)(now + (0xffffffff- lastUpdate)) / 2000000.0f);   //  uint 0.5s
//    }
//    else	
//    {
//        halfT =  ((float)(now - lastUpdate) / 2000000.0f);
//    }
//    lastUpdate = now;	//更新时间


    //快速求平方根算法
    norm = invSqrt(ax*ax + ay*ay + az*az);       
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    //把加计的三维向量转成单位向量。
    norm = invSqrt(mx*mx + my*my + mz*mz);          
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm; 
    // compute reference direction of flux
    hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
    hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
    hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz; 
    // estimated direction of gravity and flux (v and w)
    vx = 2.0f*(q1q3 - q0q2);
    vy = 2.0f*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
    wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
    wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
        exInt = exInt + ex * Ki * halfT;
        eyInt = eyInt + ey * Ki * halfT;	
        ezInt = ezInt + ez * Ki * halfT;
        // 用叉积误差来做PI修正陀螺零偏
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
    }
    // 四元数微分方程
    tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

    // 四元数规范化
    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;
		
}
/**************************实现函数********************************************
*函数原型:	   void IMU_getValues(volatile float * values)
*功　　能:	 读取加速度 陀螺仪 磁力计 的当前值  
输入参数： 将结果存放的数组首地址
加速度值：原始数据，-8192-+8192
角速度值：deg/s
磁力计值：原始数据
输出参数：没有
*******************************************************************************/
void IMU_getValues(void) {  

	IMU_Get_Raw_Data();
	MPU6500_DataSave(imu.raw.ax,imu.raw.ay,imu.raw.az,imu.raw.gx,imu.raw.gy,imu.raw.gz); 
	AK8963_DataSave(imu.raw.mx, imu.raw.my, imu.raw.mz);
	
	imu.raw.ax=(float) MPU6050_FIFO[0][10];
	imu.raw.ay=(float) MPU6050_FIFO[1][10];
	imu.raw.az=(float) MPU6050_FIFO[2][10];
	
	imu.raw.gx=(float) MPU6050_FIFO[3][10];
	imu.raw.gy=(float) MPU6050_FIFO[4][10];
	imu.raw.gz=(float) MPU6050_FIFO[5][10];

	imu.raw.mx=(float) AK8963_FIFO[0][10];
	imu.raw.my=(float) AK8963_FIFO[1][10];
	imu.raw.mz=(float) AK8963_FIFO[2][10];
	
//	//+-2g  16384  单位:g
//	imu.rip.ax=imu.raw.ax/16384;
//	imu.rip.ay=imu.raw.ay/16384;
//	imu.rip.az=imu.raw.az/16384;
	
	//+-2000   16.4  单位:rad/s
	imu.rip.gx=imu.raw.gx/16.4f/57.3f;
	imu.rip.gy=imu.raw.gy/16.4f/57.3f;
	imu.rip.gz=imu.raw.gz/16.4f/57.3f;
	
//	//单位:高斯
//	imu.rip.mx=imu.raw.mx/1000;
//	imu.rip.my=imu.raw.my/1000;
//	imu.rip.mz=imu.raw.mz/1000;
//	
//	//单位:摄氏度
//	imu.rip.temp=21+((double)imu.raw.temp)/333.87;

}

volatile  float yaw_temp,pitch_temp,roll_temp;
volatile  float last_yaw_temp,last_pitch_temp,last_roll_temp;
static int yaw_count = 0;
void GetPitchYawGxGyGz()
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
 
}//以上的三个angle是陀螺仪读出的最终三轴数据

void Recorret_offset_g(void)
{
	float sum[3]={0,0,0};
	int i=0;
	while(i<1000)
	{
		i++;
    IMU_Get_Raw_Data();
		sum[0]+=imu.raw.gx;
		sum[1]+=imu.raw.gy;
		sum[2]+=imu.raw.gz;	
	}
	imu.offset.gx =(int16_t) (sum[0]/1000);
	imu.offset.gy =(int16_t) (sum[1]/1000);
	imu.offset.gz =(int16_t) (sum[2]/1000);
}

//// System constants
//#define deltat 0.001f // sampling period in seconds (shown as 1 ms)
//#define gyroMeasError 3.14159265358979f * (5.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
//#define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta
//// Global system variables
////float ax, ay, az; // accelerometer measurements
////float gx, gy, gz; // gyroscope measurements in rad/s
//float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f; // estimated orientation quaternion elements with initial conditions
//void filterUpdate(float gx, float gy, float gz, float ax, float ay, float az)
//{
//// Local system variables
//float norm; // vector norm
//float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
//float f_1, f_2, f_3; // objective function elements
//float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
//float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
//// Axulirary variables to avoid reapeated calcualtions
//float halfSEq_1 = 0.5f * SEq_1;
//float halfSEq_2 = 0.5f * SEq_2;
//float halfSEq_3 = 0.5f * SEq_3;
//float halfSEq_4 = 0.5f * SEq_4;
//float twoSEq_1 = 2.0f * SEq_1;
//float twoSEq_2 = 2.0f * SEq_2;
//float twoSEq_3 = 2.0f * SEq_3;
//// Normalise the accelerometer measurement
//norm = sqrt(ax * ax + ay * ay + az * az);
//ax /= norm;
//ay /= norm;
//az /= norm;
//// Compute the objective function and Jacobian
//f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - ax;
//f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - ay;
//f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - az;
//J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
//J_12or23 = 2.0f * SEq_4;
//J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
//J_14or21 = twoSEq_2;
//J_32 = 2.0f * J_14or21; // negated in matrix multiplication
//J_33 = 2.0f * J_11or24; // negated in matrix multiplication
//// Compute the gradient (matrix multiplication)
//SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
//SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
//SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
//SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
//// Normalise the gradient
//norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
//SEqHatDot_1 /= norm;
//SEqHatDot_2 /= norm;
//SEqHatDot_3 /= norm;
//SEqHatDot_4 /= norm;
//// Compute the quaternion derrivative measured by gyroscopes
//SEqDot_omega_1 = -halfSEq_2 * gx - halfSEq_3 * gy - halfSEq_4 * gz;
//SEqDot_omega_2 = halfSEq_1 * gx + halfSEq_3 * gz - halfSEq_4 * gy;
//SEqDot_omega_3 = halfSEq_1 * gy - halfSEq_2 * gz + halfSEq_4 * gx;
//SEqDot_omega_4 = halfSEq_1 * gz + halfSEq_2 * gy - halfSEq_3 * gx;
//// Compute then integrate the estimated quaternion derrivative
//SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
//SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
//SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
//SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
//// Normalise quaternion
//norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
//q0=SEq_1 /= norm;
//q1=SEq_2 /= norm;
//q2=SEq_3 /= norm;
//q3=SEq_4 /= norm;

//q0=SEq_1 ;
//q1=SEq_2 ;
//q2=SEq_3 ;
//q3=SEq_4 ;

//}
