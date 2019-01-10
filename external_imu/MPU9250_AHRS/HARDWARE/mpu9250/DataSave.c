#include "DataSave.h"

IMUDataTypedef IMU_Real_Data ={0,0,0,0,0,0,0,0,0};
IMUDataTypedef IMU_RAW_Data  ={0,0,0,0,0,0,0,0,0};
int16_t MPU6050_FIFO[6][11] = {0};//[0]-[9]为最近10次数据 [10]为10次数据的平均值
int16_t AK8963_FIFO[3][11] = {0};//[0]-[9]为最近10次数据 [10]为10次数据的平均值 注：磁传感器的采样频率慢，所以单独列出

float AK8963_lastx,AK8963_lasty,AK8963_lastz;

/**********************************************************************************/
/*将MPU6050_ax,MPU6050_ay, MPU6050_az,MPU6050_gx, MPU6050_gy, MPU6050_gz处理后存储*/
/**********************************************************************************/

void MPU6050_DataSave(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz) //[0]-[9]为最近10次数据 [10]为10次数据的平均值
{
	uint8_t i = 0;
	int32_t sum=0;
	
	for(i=1;i<10;i++)
	{
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
	
	for(i=0;i<10;i++)//求当前数组的合，再取平均值
	{	
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

int16_t MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz
				,MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz;
void MPU6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
	{
  
	//if(isMPU6050_is_DRY)
	//{
	//	isMPU6050_is_DRY = 0;
	  IMU_Get_Raw_Data();
//	  GetAK8963_RawValues();
		MPU6050_Lastax=imu_data.ax;
		MPU6050_Lastay=imu_data.ay;
		MPU6050_Lastaz=imu_data.az;
		//跳过温度ADC
		MPU6050_Lastgx=imu_data.gx;
		MPU6050_Lastgy=imu_data.gy;
		MPU6050_Lastgz=imu_data.gz;		
		MPU6050_DataSave(MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz,MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz);  		
		*ax = MPU6050_FIFO[0][10];
		*ay = MPU6050_FIFO[1][10];
		*az = MPU6050_FIFO[2][10];
		*gx = MPU6050_FIFO[3][10];
		*gy = MPU6050_FIFO[4][10];
		*gz = MPU6050_FIFO[5][10];
//	} 
//	else
//	{       //读取上一次的值
//		*ax = MPU6050_FIFO[0][10];//=MPU6050_FIFO[0][10];
//		*ay = MPU6050_FIFO[1][10];//=MPU6050_FIFO[1][10];
//		*az = MPU6050_FIFO[2][10];//=MPU6050_FIFO[2][10];
//		*gx = MPU6050_FIFO[3][10]-GyroSavedCaliData.GyroXOffset;//=MPU6050_FIFO[3][10];
//		*gy = MPU6050_FIFO[4][10]-GyroSavedCaliData.GyroYOffset;//=MPU6050_FIFO[4][10];
//		*gz = MPU6050_FIFO[5][10]-GyroSavedCaliData.GyroZOffset;//=MPU6050_FIFO[5][10];
//	}
}

void  AK8963_newValues(int16_t x,int16_t y,int16_t z)
{
	uint8_t i = 0;
	int32_t sum=0;

	for(i=1;i<10;i++)
	{
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

void AK8963_getRaw(int16_t *x,int16_t *y,int16_t *z) 
{;
    AK8963_newValues(imu_data.mx, imu_data.my, imu_data.mz);
    *x = AK8963_FIFO[0][10];
    *y = AK8963_FIFO[1][10];
    *z = AK8963_FIFO[2][10];
}

void AK8963_mgetValues(volatile float *arry) 
{
    int16_t xr,yr,zr;
    AK8963_getRaw(&xr, &yr, &zr);
    arry[0]= AK8963_lastx=(float)xr;//AK8963_lastx=((float)(xr - MagSavedCaliData.MagXOffset)) * MagSavedCaliData.MagXScale;
    arry[1]= AK8963_lasty=(float)yr;//AK8963_lasty=((float)(yr - MagSavedCaliData.MagYOffset)) * MagSavedCaliData.MagYScale;
    arry[2]= AK8963_lastz=(float)zr;//AK8963_lastz=((float)(zr - MagSavedCaliData.MagZOffset)) * MagSavedCaliData.MagZScale;
}

void AK8963_getlastValues(int16_t *x,int16_t *y,int16_t *z) 
{
    *x = AK8963_FIFO[0][10];
    *y = AK8963_FIFO[1][10]; 
    *z = AK8963_FIFO[2][10]; 
}

void Recorret_offset_g(void)
{
	float sum[3]={0,0,0};
	int i=0;
	while(i<1000)
	{
		i++;
    IMU_Get_Raw_Data();
		sum[0]+=imu_data.gx;
		sum[1]+=imu_data.gy;
		sum[2]+=imu_data.gz;	
	}
	imu_data_offest.gx =(int16_t) (sum[0]/1000);
	imu_data_offest.gy =(int16_t) (sum[1]/1000);
	imu_data_offest.gz =(int16_t) (sum[2]/1000);
}
