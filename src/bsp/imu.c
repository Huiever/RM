#include "imu.h"
#include "spi.h"
#include "mpu6500_reg.h"
#include "IST8310_reg.h"
#include "delay.h"
#include "usart3.h"
#include "timer.h"
#include <math.h>
#include <string.h>
#include "adc.h"
#include "main.h"
#include "pid_regulator.h"

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
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer

                                                                                                    
imu_t                                  imu = {
                                            {0,0,0,0,0,0,0,0,0,0},       //raw
                                            {0,0,0,0,0,0,
                                                {-10.55f, 0.83f,11.92f,1.00f,-0.003f, -0.049f,1.021f,-0.015f,0.982f}
                                            },     //offset
                                            {0,0,0,0,0,0,0,0,0,0,0,0,0}  //rip
                                            };

int32_t MPU6500_FIFO[6][11] = {0};    //[0]-[9]为最近10次数据 [10]为10次数据的平均值
int16_t IST8310_FIFO[3][11] = {0};    //[0]-[9]为最近10次数据 [10]为10次数据的平均值 
                                                                        //注：磁传感器的采样频率慢，所以单独列出
uint8_t MPU_id = 0x70;

void mpu_offset_call(void);
void init_quaternion(void);

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
//Write a register to MPU6500
uint8_t MPU6500_Write_Regs(uint8_t const reg, uint8_t const *pData, uint8_t len)
{
    static uint8_t MPU_Tx;
    MPU6500_NSS_Low();
    MPU_Tx = reg&0x7f;
    SPI_ReadWriteByte(MPU_Tx);
    for(int i = 0;i<len;i++)
    {
        SPI_ReadWriteByte(pData[i]);
    }
    MPU6500_NSS_High();
    return 0;
}

uint8_t MPU6500_Write_Regss(uint8_t const addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    MPU6500_Write_Regs(reg, buf, len);
    return 0;
}

//Read a register from MPU6500
uint8_t MPU6500_Read_Reg(uint8_t const reg)
{
  static uint8_t MPU_Rx, MPU_Tx; 
  MPU6500_NSS_Low();
  MPU_Tx = reg|0x80;
  SPI_ReadWriteByte(MPU_Tx);
    MPU_Rx = SPI_ReadWriteByte(0x00);//0xff
  MPU6500_NSS_High();
  return MPU_Rx;
}
//Read registers from MPU6500,address begin with regAddr
uint8_t MPU6500_Read_Regs(uint8_t const reg, uint8_t *pData, uint8_t len)
{
    static uint8_t  MPU_Tx;//MPU_Rx;//
    MPU6500_NSS_Low();
    MPU_Tx = reg|0x80;
    SPI_ReadWriteByte(MPU_Tx);
    for(int i = 0;i<len;i++)
    {
        pData[i] = SPI_ReadWriteByte(0x00);//0xff
    }
    MPU6500_NSS_High();
    return 0;
}

uint8_t MPU6500_Read_Regss(uint8_t const addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    MPU6500_Read_Regs(reg, buf, len);
    return 0;
}

//Write IST8310 register through MPU6500
static void IST_Reg_Write_By_MPU(uint8_t addr, uint8_t data){
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, addr);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, data);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  delay_ms(10);
}
static uint8_t IST_Reg_Read_By_MPU(uint8_t addr){
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
static void MPU_Auto_Read_IST_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num){
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
uint8_t MPU6500_Init(void){
    MPU6500_NSS_High();
    
    delay_ms(100);
    
    if(MPU_id  ==  MPU6500_Read_Reg(MPU6500_WHO_AM_I)){
        printf("MPU6500 init done!");
    }
    else{ 
        return 1; //error
    }
    
  uint8_t index = 0;
  uint8_t MPU6500_Init_Data[10][2] = {
    {MPU6500_PWR_MGMT_1,    0x80},      // Reset Device
    {MPU6500_PWR_MGMT_1,    0x03},      // Clock Source - Gyro-Z
    {MPU6500_PWR_MGMT_2,    0x00},      // Enable Acc & Gyro
    {MPU6500_CONFIG,        0x04},      // LPF 98Hz
    {MPU6500_GYRO_CONFIG,   0x00},      // +-250dps
    {MPU6500_ACCEL_CONFIG,  0x00},      // +-2G
    {MPU6500_ACCEL_CONFIG_2,0x02},      // enable LowPassFilter  Set Acc LPF
    {MPU6500_USER_CTRL,     0x20},      // Enable AUX
  };
  
    for(index = 0; index < 10; index++){
        MPU6500_Write_Reg(MPU6500_Init_Data[index][0], MPU6500_Init_Data[index][1]);
        delay_ms(1);
    }
    
    MPU6500_Set_Gyro_Fsr(2);
    MPU6500_Set_Accel_Fsr(0);

    return 0;
}

uint8_t IST8310_Init(void){
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
  if(IST8310_DEVICE_ID_A !=   IST_Reg_Read_By_MPU(IST8310_WHO_AM_I))
    return 1; //error
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x01); 
  delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFA, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFA) !=   0x00)
    return 2;
  delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFB) !=   0x00)
    return 3;
  delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_AVGCNTL, 0x24);
  if(IST_Reg_Read_By_MPU(IST8310_AVGCNTL) !=   0x24)
    return 4;
  delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_PDCNTL, 0xc0);
  if(IST_Reg_Read_By_MPU(IST8310_PDCNTL) !=   0xc0)
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
void calibrateMPU6050(float * dest1, float * dest2);
void imu_init(void){
    //calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
    while(MPU6500_Init()){
        printf("MPU6500 init error！");
    }
    
    while(IST8310_Init()){
        printf("IST8310 init error！");
    }
    
    mpu_offset_call();
    init_quaternion();
}


void GetIST8310_RawValues(uint8_t* buff)
{
    MPU6500_Read_Regs(MPU6500_EXT_SENS_DATA_00,buff,6);
}

/**********************************************************************************/
/*将MPU6500_ax,MPU6500_ay, MPU6500_az,MPU6500_gx, MPU6500_gy, MPU6500_gz处理后存储*/
/**********************************************************************************/

//[0]-[9]为最近10次数据 [10]为10次数据的平均值
void mpu6500_datasave(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz){

    for(uint8_t i = 1;i<10;i++){
        MPU6500_FIFO[0][i-1] = MPU6500_FIFO[0][i];
        MPU6500_FIFO[1][i-1] = MPU6500_FIFO[1][i];
        MPU6500_FIFO[2][i-1] = MPU6500_FIFO[2][i];
        MPU6500_FIFO[3][i-1] = MPU6500_FIFO[3][i];
        MPU6500_FIFO[4][i-1] = MPU6500_FIFO[4][i];
        MPU6500_FIFO[5][i-1] = MPU6500_FIFO[5][i];
    }
    
    MPU6500_FIFO[0][9] = ax;
    MPU6500_FIFO[1][9] = ay;
    MPU6500_FIFO[2][9] = az;
    MPU6500_FIFO[3][9] = gx;
    MPU6500_FIFO[4][9] = gy;
    MPU6500_FIFO[5][9] = gz;
    
    for(uint8_t j = 0;j<6;j++){
            for(uint8_t i = 0;i<10;i++){
                MPU6500_FIFO[j][10] += MPU6500_FIFO[j][i];
            }
            MPU6500_FIFO[j][10] = MPU6500_FIFO[j][10]/10;
    }
}

void IST8310_datasave(int16_t mx,int16_t my,int16_t mz){

    for(uint8_t i = 1;i<10;i++){
        IST8310_FIFO[0][i-1] = IST8310_FIFO[0][i];
        IST8310_FIFO[1][i-1] = IST8310_FIFO[1][i];
        IST8310_FIFO[2][i-1] = IST8310_FIFO[2][i];
    }
    IST8310_FIFO[0][9] =  mx;//将新的数据放置到 数据的最后面
    IST8310_FIFO[1][9] =  my;
    IST8310_FIFO[2][9] =  mz;
    
    for(uint8_t j = 0;j<3;j++){
        for(uint8_t i = 0;i<10;i++){
            IST8310_FIFO[j][10] += IST8310_FIFO[j][i];
        }
        IST8310_FIFO[j][10] = IST8310_FIFO[j][10]/10;
    }
}

void imu_calibrate(void){
    
    float tempx,tempy,tempz;
    tempx = (float)(imu.raw.mx -imu.offset.mag.mx);
    tempy = (float)(imu.raw.my -imu.offset.mag.my);
    tempz = (float)(imu.raw.mz -imu.offset.mag.mz);
    #if ELLIPSOID_FIT
        imu.raw.mx = (int16_t)(imu.offset.mag.b0*tempx+imu.offset.mag.b1*tempy+imu.offset.mag.b2*tempz);
        imu.raw.my = (int16_t)(imu.offset.mag.b1*tempx+imu.offset.mag.b3*tempy+imu.offset.mag.b4*tempz);
        imu.raw.mz = (int16_t)(imu.offset.mag.b2*tempx+imu.offset.mag.b4*tempy+imu.offset.mag.b5*tempz);
    #else
        imu.raw.mx = (int16_t) tempx;
        imu.raw.my = (int16_t) tempy;
        imu.raw.mz = (int16_t) tempz;
    #endif
    imu.rip.mx = (float)(imu.raw.mx*0.3f);
    imu.rip.my = (float)(imu.raw.my*0.3f);
    imu.rip.mz = (float)(imu.raw.mz*0.3f);
    
    imu.raw.ax -=  imu.offset.ax;
    imu.raw.ay -=  imu.offset.ay;
    imu.raw.az -=  imu.offset.az;
    
    imu.raw.gx -=  imu.offset.gx;
    imu.raw.gy -=  imu.offset.gy;
    imu.raw.gz -=  imu.offset.gz;
}



// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU6050(float * dest1, float * dest2)
{  
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

// reset device, reset all registers, clear gyro and accelerometer bias registers

  MPU6500_Write_Reg(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay_ms(100);
   
// get stable time source
// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  MPU6500_Write_Reg(PWR_MGMT_1, 0x01);
  MPU6500_Write_Reg(PWR_MGMT_2, 0x00);
  delay_ms(200);
  
// Configure device for bias calculation

  MPU6500_Write_Reg(INT_ENABLE, 0x00);
  MPU6500_Write_Reg(FIFO_EN, 0x00);
  MPU6500_Write_Reg(PWR_MGMT_1, 0x00);
  MPU6500_Write_Reg(I2C_MST_CTRL, 0x00);
  MPU6500_Write_Reg(USER_CTRL, 0x00);
  MPU6500_Write_Reg(USER_CTRL, 0x0C);


//  writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
//  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
//  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
//  writeByte(MPU6050_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
//  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
//  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay_ms(15);
  
// Configure MPU6050 gyro and accelerometer for bias calculation
  MPU6500_Write_Reg(CONFIG, 0x01);
  MPU6500_Write_Reg(SMPLRT_DIV, 0x00);
  MPU6500_Write_Reg(GYRO_CONFIG, 0x00);
  MPU6500_Write_Reg(ACCEL_CONFIG, 0x00);


//  writeByte(MPU6050_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
//  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
//  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
//  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
// 
  uint16_t  gyrosensitivity  = 32.8;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation

  MPU6500_Write_Reg(USER_CTRL, 0x40);
  MPU6500_Write_Reg(FIFO_EN, 0x78);

//  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO  
//  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
  delay_ms(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

// At end of sample accumulation, turn off FIFO sensor read

  MPU6500_Write_Reg(FIFO_EN, 0x00);
  MPU6500_Read_Regs(FIFO_COUNTH,&data[0],2);

//  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
//  readBytes(MPU6050_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    
    MPU6500_Read_Regs(FIFO_R_W,&data[0],12);
//    readBytes(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
            
}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
 
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

// Push gyro biases to hardware registers


//  MPU6500_Write_Reg(INT_ENABLE, 0x00);
//  MPU6500_Write_Reg(FIFO_EN, 0x00);
//  MPU6500_Write_Reg(PWR_MGMT_1, 0x00);
//  MPU6500_Write_Reg(I2C_MST_CTRL, 0x00);
//  MPU6500_Write_Reg(USER_CTRL, 0x00);
//  MPU6500_Write_Reg(USER_CTRL, 0x00);


  MPU6500_Write_Reg( XG_OFFS_USRH, data[0]); 
  MPU6500_Write_Reg( XG_OFFS_USRL, data[1]);
  MPU6500_Write_Reg( YG_OFFS_USRH, data[2]);
  MPU6500_Write_Reg( YG_OFFS_USRL, data[3]);
  MPU6500_Write_Reg( ZG_OFFS_USRH, data[4]);
  MPU6500_Write_Reg( ZG_OFFS_USRL, data[5]);

  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  MPU6500_Read_Regs(XA_OFFSET_H,&data[0],2);
//  readBytes(MPU6050_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];

  MPU6500_Read_Regs(YA_OFFSET_H,&data[0],2);
//  readBytes(MPU6050_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];

  MPU6500_Read_Regs(ZA_OFFSET_H,&data[0],2);
//  readBytes(MPU6050_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
 
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Push accelerometer biases to hardware registers
//  writeByte(MPU6050_ADDRESS, XA_OFFSET_H, data[0]);  
//  writeByte(MPU6050_ADDRESS, XA_OFFSET_L_TC, data[1]);
//  writeByte(MPU6050_ADDRESS, YA_OFFSET_H, data[2]);
//  writeByte(MPU6050_ADDRESS, YA_OFFSET_L_TC, data[3]);  
//  writeByte(MPU6050_ADDRESS, ZA_OFFSET_H, data[4]);
//  writeByte(MPU6050_ADDRESS, ZA_OFFSET_L_TC, data[5]);

// Output scaled accelerometer biases for manual subtraction in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

//Get 6 axis data from MPU6500
void IMU_Get_Raw_Data(void)
{
    MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

    imu.raw.ax = (int16_t)(mpu_buff[0] << 8 | mpu_buff[1]);
    imu.raw.ay = (int16_t)(mpu_buff[2] << 8 | mpu_buff[3]);
    imu.raw.az = (int16_t)(mpu_buff[4] << 8 | mpu_buff[5]);

    imu.raw.temp = (int16_t)(mpu_buff[6] << 8 | mpu_buff[7]);

    imu.raw.gx = (int16_t)(((mpu_buff[8]  << 8 | mpu_buff[9])  - imu.offset.gx));
    imu.raw.gy = (int16_t)(((mpu_buff[10] << 8 | mpu_buff[11]) - imu.offset.gy));
    imu.raw.gz = (int16_t)(((mpu_buff[12] << 8 | mpu_buff[13]) - imu.offset.gz));

    GetIST8310_RawValues(ist_buff);
    memcpy(&imu.raw.mx, ist_buff, 6);

    mpu6500_datasave(imu.raw.ax,imu.raw.ay,imu.raw.az,imu.raw.gx,imu.raw.gy,imu.raw.gz); 
    IST8310_datasave(imu.raw.mx, imu.raw.my, imu.raw.mz);

    imu.raw.ax = (int16_t)MPU6500_FIFO[0][10];
    imu.raw.ay = (int16_t)MPU6500_FIFO[1][10];
    imu.raw.az = (int16_t)MPU6500_FIFO[2][10];

    imu.raw.gx = (int16_t)MPU6500_FIFO[3][10];
    imu.raw.gy = (int16_t)MPU6500_FIFO[4][10];
    imu.raw.gz = (int16_t)MPU6500_FIFO[5][10];

    imu.raw.mx = (int16_t)IST8310_FIFO[0][10];
    imu.raw.my = (int16_t)IST8310_FIFO[1][10];
    imu.raw.mz = (int16_t)IST8310_FIFO[2][10];

    imu_calibrate();

//    imu.raw.ax -=  accelBias[0];
//    imu.raw.ay -=  accelBias[1];
//    imu.raw.az -=  accelBias[2];

//    imu.raw.gx -=  gyroBias[0];
//    imu.raw.gy -=  gyroBias[1];
//    imu.raw.gz -=  gyroBias[2];

//    imu.rip.temp = 21 + imu.raw.temp / 333.87f;

    imu.rip.temp = imu.raw.temp * MPU6500_TEMPERATURE_FACTOR + MPU6500_TEMPERATURE_OFFSET;

//unit:m/s2
//    imu.rip.ax = (float)(imu.raw.ax * 9.8 / 16384);
//    imu.rip.ay = (float)(imu.raw.ay * 9.8 / 16384);
//    imu.rip.az = (float)(imu.raw.az * 9.8 / 16384);

    imu.rip.ax = (float)(imu.raw.ax / 16384);
    imu.rip.ay = (float)(imu.raw.ay / 16384);
    imu.rip.az = (float)(imu.raw.az / 16384);

    static uint8_t updata_count=0;
    //加速度计低通滤波
    static double accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
    static double accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
    static double accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
    static const double fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

    if(updata_count==0)
    {
        accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = imu.rip.ax;
        accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = imu.rip.ay;
        accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = imu.rip.az;
        updata_count++;
    }
    else
    {
        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];

        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + imu.rip.ax * fliter_num[2];

        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];

        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + imu.rip.ay * fliter_num[2];

        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];

        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + imu.rip.az * fliter_num[2];
    }
    
//    imu.rip.ax = accel_fliter_3[0];
//    imu.rip.ay = accel_fliter_3[1];
//    imu.rip.az = accel_fliter_3[2];
    
    /* +-1000dps -> rad/s */
    imu.rip.gx = (float)(imu.raw.gx /32.8f /57.3f);
    imu.rip.gy = (float)(imu.raw.gy /32.8f /57.3f);
    imu.rip.gz = (float)(imu.raw.gz /32.8f /57.3f);
}

void mpu_offset_call(void){

    int i;
    for (i = 0; i<300;i++){
        MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

        imu.offset.ax +=  mpu_buff[0] << 8 | mpu_buff[1];
        imu.offset.ay +=  mpu_buff[2] << 8 | mpu_buff[3];
        imu.offset.az +=  mpu_buff[4] << 8 | mpu_buff[5];

        imu.offset.gx +=  mpu_buff[8]  << 8 | mpu_buff[9];
        imu.offset.gy +=  mpu_buff[10] << 8 | mpu_buff[11];
        imu.offset.gz +=  mpu_buff[12] << 8 | mpu_buff[13];

        delay_ms(5);
    }
    for(i = 0;i<20;i++){
        GetIST8310_RawValues(ist_buff);
        memcpy(&imu.raw.mx, ist_buff, 6);
        IST8310_datasave(imu.raw.mx, imu.raw.my, imu.raw.mz);
    }

    imu.offset.ax = imu.offset.ax / 300;
    imu.offset.ay = imu.offset.ay / 300;
    imu.offset.az = imu.offset.az / 300;
    imu.offset.gx = imu.offset.gx / 300;
    imu.offset.gy = imu.offset.gx / 300;
    imu.offset.gz = imu.offset.gz / 300;
    //用来初始化四元数
    imu.raw.mx = (int16_t)IST8310_FIFO[0][10];
    imu.raw.my = (int16_t)IST8310_FIFO[1][10];
    imu.raw.mz = (int16_t)IST8310_FIFO[2][10];
}

void init_quaternion(void){
    int16_t hx, hy;//hz;
    
    hx = imu.raw.mx;
    hy = imu.raw.my;
    //hz = imu.raw.mz;
    
    #ifdef BOARD_DOWN
    if (hx < 0 && hy < 0) 
    {
        if (fabs(hx / hy) >=  1)
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
        if (fabs(hx / hy) >= 1)
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
        if (fabs(hx / hy) >=  1)
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
        if (fabs(hx / hy) >=  1)
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
        if (fabs(hx / hy) > =  1)
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
        if(fabs(hx / hy) > =  1)
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
        if(fabs(hx / hy) > =  1)
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
        if(fabs(hx / hy) > =  1)
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


void IMU_AHRSupdate(void){
    float norm,halfT;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
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
//    ax = imu.raw.ax;
//    ay = imu.raw.ay;
//    az = imu.raw.az;
//    mx = imu.raw.mx;
//    my = imu.raw.my;
//    mz = imu.raw.mz;
    ax = imu.rip.ax;
    ay = imu.rip.ay;
    az = imu.rip.az;
    mx = imu.rip.mx;
    my = imu.rip.my;
    mz = imu.rip.mz;

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
    if(ex !=   0.0f && ey !=   0.0f && ez !=   0.0f)
    {
        exInt = exInt + ex * Ki * halfT;
        eyInt = eyInt + ey * Ki * halfT;
        ezInt = ezInt + ez * Ki * halfT;
        
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
    }
    
    tempq0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
    tempq1 = q1 + ( q0*gx + q2*gz - q3*gy) * halfT;
    tempq2 = q2 + ( q0*gy - q1*gz + q3*gx) * halfT;
    tempq3 = q3 + ( q0*gz + q1*gy - q2*gx) * halfT;

    /* normalise quaternion */
    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;
}

// =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  = 
// MahonyAHRS.c
// =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  = 
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date            Author            Notes
// 29/09/2011    SOH Madgwick    Initial release
// 02/10/2011    SOH Madgwick    Optimised for reduced CPU load
//
// =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  = 

//---------------------------------------------------------------------------------------------------
// Header files


#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq    100.0f            // sample frequency in Hz
#define twoKpDef    (100.0f * 0.5f)    // 2 * proportional gain
#define twoKiDef    0    // 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;                                            // 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;                                            // 2 * integral gain (Ki)
//volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;                    // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;    // integral error terms scaled by Ki

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax  ==  0.0f) && (ay  ==  0.0f) && (az  ==  0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *=  recipNorm;
        ay *=  recipNorm;
        az *=  recipNorm;        

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;
    
        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            integralFBx +=  twoKi * halfex * (1.0f / sampleFreq);    // integral error scaled by Ki
            integralFBy +=  twoKi * halfey * (1.0f / sampleFreq);
            integralFBz +=  twoKi * halfez * (1.0f / sampleFreq);
            gx +=  integralFBx;    // apply integral feedback
            gy +=  integralFBy;
            gz +=  integralFBz;
        }
        else {
            integralFBx = 0.0f;    // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx +=  twoKp * halfex;
        gy +=  twoKp * halfey;
        gz +=  twoKp * halfez;
    }
    
    // Integrate rate of change of quaternion
    gx *=  (0.5f * (1.0f / sampleFreq));        // pre-multiply common factors
    gy *=  (0.5f * (1.0f / sampleFreq));
    gz *=  (0.5f * (1.0f / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 +=  (-qb * gx - qc * gy - q3 * gz);
    q1 +=  (qa * gx + qc * gz - q3 * gy);
    q2 +=  (qa * gy - qb * gz + q3 * gx);
    q3 +=  (qa * gz + qb * gy - qc * gx); 
    
    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *=  recipNorm;
    q1 *=  recipNorm;
    q2 *=  recipNorm;
    q3 *=  recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

// =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  = 
// END OF CODE
// =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  = 

// parameters for 6 DoF sensor fusion calculations
#define PI = 3.14159265358979323846f;
#define GyroMeasError = 1.04719755119;     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
volatile float beta = 0.9068996821;  // compute beta
#define GyroMeasDrift = PI * (1.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
volatile float zeta = 0.0151149947;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
volatile float deltat = 0.0f;                              // integration interval for both filter schemes
volatile int lastUpdate = 0, firstUpdate = 0, Now = 0;     // used to calculate integration interval                               // used to calculate integration interval

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration and rotation rate to produce a quaternion-based estimate of relative
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz)
{
    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objective funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;  // gyro bias error

    // Auxiliary variables to avoid repeated arithmetic
    float _halfq1 = 0.5f * q0;
    float _halfq2 = 0.5f * q1;
    float _halfq3 = 0.5f * q2;
    float _halfq4 = 0.5f * q3;
    float _2q1 = 2.0f * q0;
    float _2q2 = 2.0f * q1;
    float _2q3 = 2.0f * q2;
    float _2q4 = 2.0f * q3;
//            float _2q1q3 = 2.0f * q0 * q2;
//            float _2q3q4 = 2.0f * q2 * q3;

    Now = Get_Time_Micros(); //ms
    deltat = (float)((Now - lastUpdate)/1000.0f) ; // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    if(lastUpdate - firstUpdate > 10000.0f) {
     beta = 0.04;  // decrease filter gain after stabilized
     zeta = 0.015; // increasey bias drift gain after stabilized
    }


    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;
    
    // Compute the objective function and Jacobian
    f1 = _2q2 * q3 - _2q1 * q2 - ax;
    f2 = _2q1 * q1 + _2q3 * q3 - ay;
    f3 = 1.0f - _2q2 * q1 - _2q3 * q2 - az;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;
  
    // Compute the gradient (matrix multiplication)
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;
    
    // Normalize the gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;
    
    // Compute estimated gyroscope biases
    gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
    
    // Compute and remove gyroscope biases
    gbiasx += gerrx * deltat * zeta;
    gbiasy += gerry * deltat * zeta;
    gbiasz += gerrz * deltat * zeta;
//           gx -= gbiasx;
//           gy -= gbiasy;
//           gz -= gbiasz;
    
    // Compute the quaternion derivative
    qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
    qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
    qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
    qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

    // Compute then integrate estimated quaternion derivative
    q0 += (qDot1 -(beta * hatDot1)) * deltat;
    q1 += (qDot2 -(beta * hatDot2)) * deltat;
    q2 += (qDot3 -(beta * hatDot3)) * deltat;
    q3 += (qDot4 -(beta * hatDot4)) * deltat;

    // Normalize the quaternion
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);    // normalise quaternion
    norm = 1.0f/norm;
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;
    
}


/**
    * @brief  update imu attitude
  * @param  
    * @retval 
  * @usage  call in main() function
    */
void IMU_getYawPitchRoll(void)
{
    volatile static float yaw_temp = 0,last_yaw_temp = 0;
    volatile static int   yaw_count = 0;
//    // yaw    -pi----pi
//    imu.rip.yaw = -atan2(2 * q1 * q2 + 2 * q0* q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 57.3;
//    // pitch  -pi/2--- pi/2
//    imu.rip.pit = -asin(-2 * q1 * q3 + 2 * q0 * q2)* 57.3;
//    // roll   -pi-----pi
//    imu.rip.rol = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1)* 57.3;
    // yaw    -pi----pi
    imu.rip.yaw = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3)* 57.3;
    // pitch  -pi/2--- pi/2
    imu.rip.pit = -asin(2.0f * (q1 * q3 - q0 * q2))* 57.3;
    // roll   -pi-----pi
    imu.rip.rol = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3)* 57.3;

    last_yaw_temp = yaw_temp;
    yaw_temp = imu.rip.yaw; 
    if(yaw_temp - last_yaw_temp>= 330){
        yaw_count--;
    } 
    else if (yaw_temp - last_yaw_temp <= -330){
        yaw_count++;
    }
    imu.rip.yaw = yaw_temp + yaw_count*360;
}

static uint8_t first_temperature = 0;

static int8_t IMU_GET_CONTROL_TEMPERATURE(void){
    static int8_t control_temperature = 0;
		static uint8_t count=0;
    if(count==0){
			control_temperature = (int8_t)(get_temprate()) + 10;
			if (control_temperature > (int8_t)(GYRO_CONST_MAX_TEMP))
			{
					control_temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
			}
			count=1;
		}
//		printf("%8d\r\n", control_temperature);
    return control_temperature;
}

PID_Regulator_t IMUTemperaturePID = IMU_Temperature_PID_DEFAULT;

static void IMU_temp_Control(double temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0 ;
    if (first_temperature)
    {
        IMUTemperaturePID.ref = IMU_GET_CONTROL_TEMPERATURE();
        IMUTemperaturePID.fdb = temp;
        IMUTemperaturePID.Calc(&IMUTemperaturePID);
        
        if (IMUTemperaturePID.output < 0.0f)
        {
            IMUTemperaturePID.output = 0.0f;
        }
        tempPWM = (uint16_t)IMUTemperaturePID.output;
        TIM_SetCompare2(TIM3, (tempPWM));
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        if (temp > IMU_GET_CONTROL_TEMPERATURE())
        {
            temp_constant_time ++;
            if(temp_constant_time > 200)
            {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                first_temperature = 1;
                //imuTempPid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }
        TIM_SetCompare2(TIM3, (MPU6500_TEMP_PWM_MAX - 1));
    }
}

float get_yaw_angle(void){
  return imu.rip.yaw;
}

float get_pit_angle(void){
    return imu.rip.pit;
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
    //MahonyAHRSupdateIMU(imu.rip.gx,imu.rip.gy,imu.rip.gz,imu.rip.ax,imu.rip.ay,imu.rip.az);
    //MadgwickQuaternionUpdate(imu.rip.ax,imu.rip.ay,imu.rip.az, imu.rip.gx,imu.rip.gy,imu.rip.gz);
//    IMU_temp_Control(imu.rip.temp);
    IMU_getYawPitchRoll();
    IMU_GET_CONTROL_TEMPERATURE();
    delay_ms(5);
#if Monitor_IMU_Angle == 1
    printf("yaw_angle:%8.3lf   pit_angle:%8.3lf  rol_angle:%8.3lf\r\n", imu.rip.yaw, imu.rip.pit, imu.rip.rol);
//    printf("%8.3lf,%8.3lf,%8.3lf\r\n", imu.rip.yaw, imu.rip.pit,imu.rip.yaw - 1.74*fabs(imu.rip.pit));
    delay_ms(5);
#endif
#if Monitor_IMU_Accel == 1
    printf("wx:%8.3lf  wy:%8.3lf  wz:%8.3lf\r\n", imu.rip.gx, imu.rip.gy, imu.rip.gz);
    delay_ms(5);
#endif
#if Monitor_IMU_Accel_Raw == 1
    printf("raw_gx:%8d raw_gy:%8d  raw_gz:%8d\r\n", imu.raw.gx, imu.raw.gy, imu.raw.gz);
    delay_ms(5);
#endif
#if Monitor_IMU_Mag == 1
    printf("%.3f,%.3f,%.3f\r\n",(float)imu.raw.mx/1000,(float)imu.raw.my/1000,(float)imu.raw.mz/1000);
    delay_ms(5);
#endif
}
