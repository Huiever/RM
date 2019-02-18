#include "judge_task.h"
#include <math.h>
#include "remote_task.h"
#include "control_task.h"

#include "main.h"

unsigned char judge_buffer[1000];
unsigned char switch_buffer[20];
tStudentGameProcessInfo     GameProcessInfo;
tLifeChangedInfo            LifeChangedInfo;
tStudentInterRealShootData  RealShootData;
extPowerHeatData_t          PowerHeatData;

extShowData_t Judge_Send_Data = { 0, 0, 0, 0};
uint8_t data[13];
uint8_t computer_tx_buf[100];

const u8 CRC8_INIT = 0xff;
const u8 CRC8_TAB[256] = {
	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
	0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
	0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
	0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
	0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
	0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
	0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
	0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
	0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
	0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
	0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
	0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
	0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
	0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
	0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
	0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
};

u8 Get_CRC8_Check_Sum( u8 *pchMessage, u32 dwLength, u8 ucCRC8 ){
	u8 ucIndex;
	while( dwLength-- ){
		ucIndex = ucCRC8^( *pchMessage++ );
		ucCRC8 = CRC8_TAB[ ucIndex ];
	}
	return( ucCRC8 );
}

/*
**	Descriptions:	CRC8 Verify function
**	Input:				Data to Verify,Stream length = Data + checksum
**	Output:				True or False (CRC Verify Result)
*/
u16 Verify_CRC8_Check_Sum( u8 *pchMessage, u16 dwLength )
{
	u8 ucExpected = 0;
	if( ( pchMessage == 0 ) || ( dwLength <= 2 ) )
		return 0;
	ucExpected = Get_CRC8_Check_Sum( pchMessage, dwLength-1, CRC8_INIT );
	
	return ( ucExpected == pchMessage[ dwLength-1 ] );
};	

/*
** 	Descriptions:	append CRC8 to the end of data
**	Input:				Data to CRC and append,Stream length = Data + checksum
**	Output:				True or False (CRC Verify Result)
*/ 
void Append_CRC8_Check_Sum( u8 *pchMessage, u32 dwLength )
{
	u8 ucCRC = 0;
	if ( ( pchMessage == 0 ) || ( dwLength <= 2 ) ) 
		return;
	ucCRC = Get_CRC8_Check_Sum ( ( u8 * )pchMessage, dwLength-1, CRC8_INIT );
	pchMessage[ dwLength-1 ] = ucCRC;
};  

u16 CRC_INIT = 0xffff;
const  u16  wCRC_Table[256] ={
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78 
};

/*
**	Descriptions:	CRC16 checksum function
**	Input:				Data to check,Stream length, initialized checksum
**	Output:				CRC checksum
*/ 
u16 Get_CRC16_Check_Sum( u8 *pchMessage, u32 dwLength, uint16_t wCRC )
{
	u8 chData;
	if ( pchMessage == NULL )
	{
		return 0xFFFF;
	}
	
	while( dwLength-- )
	{
		chData = *pchMessage++;
		( wCRC ) = ( ( u16 )( wCRC >> 8 )^ wCRC_Table[ ( ( u16 )( wCRC )^ ( u16 )( chData ) ) & 0x00ff ] );
	}
	
	return wCRC;
}

/*
**	Descriptions:	CRC16 Verify function
**	Input:				Data to Verify, Stream length = Data + checksum
**	Output:				True or False( CRC Verify Result )
*/

u32 Verify_CRC16_Check_Sum( u8 *pchMessage, u32 dwLength )
{
	u16 wExpected = 0; 
  if ( ( pchMessage == NULL ) || ( dwLength <= 2 ) ) 
	{ 
		return 0;	//False; 
	} 
	wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT );
	return ( ( wExpected & 0xff ) == pchMessage[dwLength - 2] && ( ( wExpected >> 8 ) & 0xff ) == pchMessage[dwLength - 1] ); 
}

/*
**	Descriptions:	append CRC16 to the end of data
**	Input:				Data to CRC and append,Stream length = Data + checksum
**	Output:				True or False (CRC Verify Result)
*/ 
void Append_CRC16_Check_Sum( u8 * pchMessage, u32 dwLength ) 
{  
	u16 wCRC = 0; 
	if ( ( pchMessage == NULL ) || ( dwLength <= 2 ) )
		return;
	wCRC = Get_CRC16_Check_Sum ( (u8 *)pchMessage, dwLength-2, CRC_INIT ); 
	pchMessage[ dwLength - 2 ] = (u8)( wCRC & 0x00ff );
	pchMessage[ dwLength - 1 ] = (u8)( ( wCRC >> 8 ) & 0x00ff );
}

void Judge_Init(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
    USART_InitTypeDef   USART_InitStructure;
    NVIC_InitTypeDef    NVIC_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
    USART_DeInit(USART2);
    // PD6 - RX, PD5 - TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART_InitStructure);
    USART_Cmd(USART2, ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    TIM3_Configuration();

    //printf("Judge_Init: Accomplished\r\n");
}

void USART2_IRQHandler(void){
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET){
        DMA_InitTypeDef		DMA_InitStructure;
        USART_InitTypeDef	USART_InitStructure;
        NVIC_InitTypeDef	NVIC_InitStructure;
        USART_DeInit(USART2);
        delay_ms(3);
        RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA1, ENABLE);
        USART_InitStructure.USART_BaudRate = 115200;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_No;
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(USART2, &USART_InitStructure);
        USART_Cmd(USART2, ENABLE);
        USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
        DMA_DeInit(DMA1_Stream5);
        DMA_InitStructure.DMA_Channel = DMA_Channel_4;
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)judge_buffer;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
        DMA_InitStructure.DMA_BufferSize = 1000;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
        DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
        DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
        DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA1_Stream5,&DMA_InitStructure);
        DMA_ITConfig(DMA1_Stream5,DMA_IT_TC,ENABLE);
        DMA_Cmd(DMA1_Stream5,ENABLE);
    }
}

void DMA1_Stream5_IRQHandler(void){
	if(DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5)){
		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
		DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
		for (u8 i = 0; i < 100; i++ ){
			if(judge_buffer[i]==0xA5&&judge_buffer[i+6]==0x00){
				if(judge_buffer[i+5]==CmdID_3){
//					printf("%d\r\n",GameProcessInfo.remainTime);
				}
				if(judge_buffer[i+5]==CmdID_1&&((i+20)<100)){
					GameProcessInfo.remainTime      =judge_buffer[i+7] | judge_buffer[i+8]<<8;
					GameProcessInfo.CurrentLevel    =judge_buffer[i+10];
					GameProcessInfo.remainLifeValue =judge_buffer[i+11] | judge_buffer[i+12]<<8;
					GameProcessInfo.MaxLifeValue    =judge_buffer[i+13] | judge_buffer[i+14]<<8;
//					printf( "%d\r\n",GameProcessInfo.remainTime);
//					printf( "%d\r\n",GameProcessInfo.CurrentLevel);
//					printf( "%d\r\n",GameProcessInfo.remainLifeValue);
//					printf( "%d\r\n",GameProcessInfo.MaxLifeValue);
				}
				else if(judge_buffer[i+5]==CmdID_2&&((i+11)<100)){
//					LifeChangedInfo.type=judge_buffer[i+7];
//					LifeChangedInfo.data=judge_buffer[i+8]|judge_buffer[i+9]<<8;
				}
				else if(judge_buffer[i+5]==CmdID_3&&((i+15)<100)){
					RealShootData.bulletType=judge_buffer[i+7];
					RealShootData.bulletFreq=judge_buffer[i+8];
					RealShootData.bulletSpeed = ( ( ( (float)judge_buffer[i+9] / 256 + (float)judge_buffer[i+10] ) / 256 + (float)(judge_buffer[i+11] & 0x7F)\
						) / 128 + 1 ) * pow(2, ( judge_buffer[i+12] & 0x7F) * 2 + (judge_buffer[i+11] & 0x80) / 128 - 127) * \
					((judge_buffer[i+12] & 0x80) == 0 ? 1 : -1);
//					printf( "%d\r\n",RealShootData.bulletType);
//					printf( "%d\r\n",RealShootData.bulletFreq);
//					printf( "%f\r\n",RealShootData.bulletSpeed);
				}
				else if(judge_buffer[i+5]==CmdID_4&&((i+30)<100)){
					PowerHeatData.chassisVolt = ( ( ( (float)judge_buffer[i+7] / 256 + (float)judge_buffer[i+8] ) / 256 + (float)(judge_buffer[i+9] & 0x7F)\
						) / 128 + 1 ) * pow(2, ( judge_buffer[i+10] & 0x7F) * 2 + (judge_buffer[i+9] & 0x80) / 128 - 127) * \
					((judge_buffer[i+10] & 0x80) == 0 ? 1 : -1);
						PowerHeatData.chassisCurrent = ( ( ( (float)judge_buffer[i+11] / 256 + (float)judge_buffer[i+12] ) / 256 + (float)(judge_buffer[i+13] & 0x7F)\
						) / 128 + 1 ) * pow(2, ( judge_buffer[i+14] & 0x7F) * 2 + (judge_buffer[i+3] & 0x80) / 128 - 127) * \
					((judge_buffer[i+14] & 0x80) == 0 ? 1 : -1);
							PowerHeatData.chassisPower = ( ( ( (float)judge_buffer[i+15] / 256 + (float)judge_buffer[i+16] ) / 256 + (float)(judge_buffer[i+17] & 0x7F)\
						) / 128 + 1 ) * pow(2, ( judge_buffer[i+18] & 0x7F) * 2 + (judge_buffer[i+17] & 0x80) / 128 - 127) * \
					((judge_buffer[i+18] & 0x80) == 0 ? 1 : -1);
							PowerHeatData.chassisPowerBuffer = ( ( ( (float)judge_buffer[i+19] / 256 + (float)judge_buffer[i+20] ) / 256 + (float)(judge_buffer[i+21] & 0x7F)\
						) / 128 + 1 ) * pow(2, ( judge_buffer[i+22] & 0x7F) * 2 + (judge_buffer[i+21] & 0x80) / 128 - 127) * \
					((judge_buffer[i+22] & 0x80) == 0 ? 1 : -1);
					PowerHeatData.shooterHeat17=judge_buffer[i+23]|judge_buffer[i+24]<<8;
					PowerHeatData.shooterHeat42=judge_buffer[i+24]|judge_buffer[i+25]<<8;
//					printf( "%f\r\n",PowerHeatData.chassisVolt);
//					printf( "%f\r\n",PowerHeatData.chassisCurrent);
//					printf( "%f\r\n",PowerHeatData.chassisPower);
//					printf( "%f\r\n",PowerHeatData.chassisPowerBuffer);
//					printf( "%d\r\n",PowerHeatData.shooterHeat17);
//					printf( "%d\r\n",PowerHeatData.shooterHeat42);
				}
			}
		}
	}
}

uint16_t JUDGE_GET_Infantry_HeatData(void)
{
	if (PowerHeatData.shooterHeat17 < 500)
		return PowerHeatData.shooterHeat17;
	else
		return 0;
}

uint8_t JUDGE_GET_Infantry_LevelData(void)
{
	return GameProcessInfo.CurrentLevel;
}

/**
  * @brief     pack data to bottom device
  * @param[in] cmd_id:  command id of data
  * @param[in] *p_data: pointer to the data to be sent
  * @param[in] len:     the data length
  */

void data_pack_handle(uint8_t *p_data, uint16_t len)
{
    memset(computer_tx_buf, 0, COMPUTER_FRAME_BUFLEN);
    uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;

    static uint8_t seq = 0;
    computer_tx_buf[0] = 0xa5;
    computer_tx_buf[1] = 0x0D;
    computer_tx_buf[2] = 0x00;
    computer_tx_buf[3] = seq++;
    Append_CRC8_Check_Sum(computer_tx_buf, HEADER_LEN);

    computer_tx_buf[5] = 0x00;
    computer_tx_buf[6] = 0x01;

    memcpy(&computer_tx_buf[HEADER_LEN + CMD_LEN], p_data, len);
    Append_CRC16_Check_Sum(computer_tx_buf, HEADER_LEN + CMD_LEN + len + CRC_LEN);

    int t;
    for(t=0;t<frame_length;t++)
    {
        //printf("%d\t",*( (uint8_t *)computer_tx_buf + t ));
        USART_SendData(USART2, *( computer_tx_buf + t ) );         //向串口2发送数据
        while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//等待发送结束
    }
}

void TIM3_Configuration(void){
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = 2000; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler= 8400-1;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

void TIM3_IRQHandler(void){
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
        TIM_ClearFlag(TIM3, TIM_FLAG_Update);
		
		memcpy(data, (uint8_t*)&Judge_Send_Data, 13);
		
		data_pack_handle( data, 13);
	}
}

void Set_Judge_Send_Data( uint8_t i, float Tmp_Data ){
	switch ( i )
	{
		case 1:
		{
			Judge_Send_Data.data1 = Tmp_Data;
			break;
		}
		case 2:
		{
			Judge_Send_Data.data2 = Tmp_Data;
			break;
		}
		case 3:
		{
			Judge_Send_Data.data3 = Tmp_Data;
			break;
		}
		default:
		{
			break;
		}
	}
	return;
}

void Set_Judge_Send_Flag( uint8_t i, uint8_t Tmp_Flag){
	if ( i > 5 ){
		return;
	}
	
	if ( Tmp_Flag == 0 ){
		Judge_Send_Data.mask &= ~( 0x01 << i );
	}
	else{
		Judge_Send_Data.mask |= ( 0x01 << i );
	}
	return;
}

