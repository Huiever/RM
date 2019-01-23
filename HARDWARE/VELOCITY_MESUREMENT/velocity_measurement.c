#include "velocity_measurement.h"

Flag V_Flag  =  V_FlAG_DAFAULT; //��c99ѡ�оͿ��Ա�����
const int TIMER3_FRQ=1000000;

void Get_Counter(void);
unsigned int counter=0;

//����ų�ʼ��
void IR_Init(void)   
{
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //ʹ��PA�˿�ʱ��

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;				 //������ź�IR-->PA.0 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		 //�������룬���ǲ������ã�������������������������������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //IO���ٶ�Ϊ50MHz
 GPIO_Init(GPIOA, &GPIO_InitStructure);					   //�����趨������ʼ��GPIOA.0
 GPIO_SetBits(GPIOA,GPIO_Pin_0);						       //PA.0 �����
	
}

//void Ex_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM) 
//{ 
//	u8 EXTOFFSET=(BITx%4)*4;  
//	RCC->APB2ENR|=1<<14;  						//ʹ��SYSCFGʱ��  
//	SYSCFG->EXTICR[BITx/4]&=~(0x000F<<EXTOFFSET);//���ԭ�����ã�����
//	SYSCFG->EXTICR[BITx/4]|=GPIOx<<EXTOFFSET;	//EXTI.BITxӳ�䵽GPIOx.BITx 
//	//�Զ�����
//	EXTI->IMR|=1<<BITx;					//����line BITx�ϵ��ж�(���Ҫ��ֹ�жϣ��򷴲�������)
//	if(TRIM&0x01)EXTI->FTSR|=1<<BITx;	//line BITx���¼��½��ش���
//	if(TRIM&0x02)EXTI->RTSR|=1<<BITx;	//line BITx���¼��������ش���
//}


//�ⲿ�жϳ�ʼ������
void EXTIX_Falling_Init(void)
{
 	  EXTI_InitTypeDef EXTI_InitStructure;
 	  NVIC_InitTypeDef NVIC_InitStructure;
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//�ⲿ�жϣ���Ҫʹ��AFIOʱ��

    //GPIOA.0	  �ж����Լ��жϳ�ʼ������
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);
	
   	EXTI_InitStructure.EXTI_Line=EXTI_Line0;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // �½��ش���
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

  	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//�����ȼ�1
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure);  	  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���	
}

void EXTIX_Rising_Init(void)
{
 	  EXTI_InitTypeDef EXTI_InitStructure;
 	  NVIC_InitTypeDef NVIC_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//�ⲿ�жϣ���Ҫʹ��AFIOʱ��

    //GPIOA.0	  �ж����Լ��жϳ�ʼ������
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);

   	EXTI_InitStructure.EXTI_Line=EXTI_Line0;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;   //�����ش���
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

  	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;	//��ռ���ȼ�2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//�����ȼ�1
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure);  	  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
}

//�ⲿ�жϷ�����
void EXTI0_IRQHandler(void)  
{
	Get_Counter();
	EXTI_ClearITPendingBit(EXTI_Line0);  //���EXTI0��·����λ
}

//�Լ�ģ��
uint8_t IR_Auto_Check(void)  
{
	if(IR==1) return 1;
	else      return 0;
}

//�ٶȲ��������������ٶ�ֵ
double velocity_measurement(void)  
{
	return (double)(DISTANCE/counter*TIMER3_FRQ);  
}


//����ӵ���������ż����������Ĵ���
void Get_Counter(void)
{
	if(V_Flag.across_flag==0)
	{	  
		TIM_SetCounter(TIM3,0); //TIM3->CNT=0	 
		TIM_Cmd(TIM3, ENABLE);
		V_Flag.across_flag=1;
		EXTIX_Rising_Init(); //�����ش���
	}
	else
	{
		counter=TIM_GetCounter(TIM3);
		TIM_Cmd(TIM3, DISABLE); 
		V_Flag.across_flag=0;
		V_Flag.shoot_flag=1;
		EXTIX_Falling_Init();  //�½��ش���
	}
}
