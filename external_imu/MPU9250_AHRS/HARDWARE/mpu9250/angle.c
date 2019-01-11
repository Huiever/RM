#include "angle.h"

volatile float q0 = 1.0f;
volatile float q1 = 0.0f;
volatile float q2 = 0.0f;
volatile float q3 = 0.0f;
volatile float exInt, eyInt, ezInt;  // ������
volatile float gx, gy, gz, ax, ay, az, mx, my, mz;   //��������ڴ��ļ���
volatile uint32_t lastUpdate, now; // �������ڼ��� ��λ us
int16_t MPU6050_FIFO[6][11] = {0};//[0]-[9]Ϊ���10������ [10]Ϊ10�����ݵ�ƽ��ֵ
int16_t AK8963_FIFO[3][11] = {0};//[0]-[9]Ϊ���10������ [10]Ϊ10�����ݵ�ƽ��ֵ ע���Ŵ������Ĳ���Ƶ���������Ե����г�

// Fast inverse square-root
/**************************ʵ�ֺ���********************************************
*����ԭ��:	   float invSqrt(float x)
*��������:	   ���ټ��� 1/Sqrt(x) 	
��������� Ҫ�����ֵ
��������� ���
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
/*��MPU6050_ax,MPU6050_ay, MPU6050_az,MPU6050_gx, MPU6050_gy, MPU6050_gz�����洢*/
/**********************************************************************************/

void MPU6500_DataSave(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz){ //[0]-[9]Ϊ���10������ [10]Ϊ10�����ݵ�ƽ��ֵ

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
	
	MPU6050_FIFO[0][9]=ax;//���µ����ݷ��õ� ���ݵ������
	MPU6050_FIFO[1][9]=ay;
	MPU6050_FIFO[2][9]=az;
	MPU6050_FIFO[3][9]=gx;
	MPU6050_FIFO[4][9]=gy;
	MPU6050_FIFO[5][9]=gz;
	
	for(i=0;i<10;i++){     //��ǰ����ĺϣ���ȡƽ��ֵ	
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
	AK8963_FIFO[0][9]= x;//���µ����ݷ��õ� ���ݵ������
	AK8963_FIFO[1][9]= y;
	AK8963_FIFO[2][9]= z;
	
	for(i=0;i<10;i++)//��ǰ����ĺϣ���ȡƽ��ֵ
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


/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void Init_Quaternion
*��������:	 ��ʼ����Ԫ��
��������� ��ǰ�Ĳ���ֵ��
���������û��
*******************************************************************************/
//��ʼ��IMU����
#define BOARD_DOWN 1   //�������泯�°ڷ�

void Init_Quaternion()//���ݲ������ݣ���ʼ��q0,q1,q2.q3���Ӷ��ӿ������ٶ�
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
	
	//����hx hy hz���ж�q��ֵ��ȡ�ĸ������ֵ���ƽ�����,��ʼֵ������ŷ����ת������Ԫ������õ�
	 
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_AHRSupdate
*��������:	 ����AHRS ������Ԫ�� 
��������� ��ǰ�Ĳ���ֵ��
���������û��
*******************************************************************************/
#define Kp 10.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.01f   // integral gain governs rate of convergence of gyroscope biases

void IMU_AHRSupdate(void) {

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
    gx = imu.rip.gx / 57.3f;
    gy = imu.rip.gy / 57.3f;
    gz = imu.rip.gz / 57.3f;
    mx = imu.raw.mx;
    my = imu.raw.my;
    mz = imu.raw.mz;	
	

		now  = Get_Time_Micros(); //ms
		halfT  = ((float)(now - lastUpdate) / 2000.0f);
		lastUpdate = now;


//    now = Get_Time_Micros();  //��ȡʱ�� ��λ��us   
//    if(now<lastUpdate)
//    {
//    //halfT =  ((float)(now + (0xffffffff- lastUpdate)) / 2000000.0f);   //  uint 0.5s
//    }
//    else	
//    {
//        halfT =  ((float)(now - lastUpdate) / 2000000.0f);
//    }
//    lastUpdate = now;	//����ʱ��


    //������ƽ�����㷨
    norm = invSqrt(ax*ax + ay*ay + az*az);       
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    //�ѼӼƵ���ά����ת�ɵ�λ������
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
        // �ò���������PI����������ƫ
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
    }
    // ��Ԫ��΢�ַ���
    tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

    // ��Ԫ���淶��
    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;
		
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_getValues(volatile float * values)
*��������:	 ��ȡ���ٶ� ������ ������ �ĵ�ǰֵ  
��������� �������ŵ������׵�ַ
���ٶ�ֵ��ԭʼ���ݣ�-8192-+8192
���ٶ�ֵ��deg/s
������ֵ��ԭʼ����
���������û��
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
	
//	//+-2g  16384  ��λ:g
//	imu.rip.ax=imu.raw.ax/16384;
//	imu.rip.ay=imu.raw.ay/16384;
//	imu.rip.az=imu.raw.az/16384;
	
	//+-2000   16.4  ��λ:deg/s
	imu.rip.gx=imu.raw.gx/16.4f;
	imu.rip.gy=imu.raw.gy/16.4f;
	imu.rip.gz=imu.raw.gz/16.4f;
	
//	//��λ:��˹
//	imu.rip.mx=imu.raw.mx/1000;
//	imu.rip.my=imu.raw.my/1000;
//	imu.rip.mz=imu.raw.mz/1000;
//	
//	//��λ:���϶�
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
	if(yaw_temp-last_yaw_temp>=330){  //yaw��ǶȾ����������������
		yaw_count--;
	}
	else if (yaw_temp-last_yaw_temp<=-330){
		yaw_count++;
	}
	imu.rip.yaw = yaw_temp + yaw_count*360;  //yaw��Ƕ�
 
}//���ϵ�����angle�������Ƕ�����������������

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

