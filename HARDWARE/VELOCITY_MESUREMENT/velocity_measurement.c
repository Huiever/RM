#include "velocity_measurement.h"

Flag V_Flag  =  V_FlAG_DAFAULT; //把c99选中就可以编译了
const int TIMER3_FRQ=1000000;

void Get_Counter(void);
unsigned int counter=0;

//光电门初始化
void IR_Init(void)   
{
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //使能PA端口时钟

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;				 //光电门信号IR-->PA.0 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		 //下拉输入，但是不起作用！！！！！！！！！！！！！！！！
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //IO口速度为50MHz
 GPIO_Init(GPIOA, &GPIO_InitStructure);					   //根据设定参数初始化GPIOA.0
 GPIO_SetBits(GPIOA,GPIO_Pin_0);						       //PA.0 输出高
	
}

//void Ex_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM) 
//{ 
//	u8 EXTOFFSET=(BITx%4)*4;  
//	RCC->APB2ENR|=1<<14;  						//使能SYSCFG时钟  
//	SYSCFG->EXTICR[BITx/4]&=~(0x000F<<EXTOFFSET);//清除原来设置！！！
//	SYSCFG->EXTICR[BITx/4]|=GPIOx<<EXTOFFSET;	//EXTI.BITx映射到GPIOx.BITx 
//	//自动设置
//	EXTI->IMR|=1<<BITx;					//开启line BITx上的中断(如果要禁止中断，则反操作即可)
//	if(TRIM&0x01)EXTI->FTSR|=1<<BITx;	//line BITx上事件下降沿触发
//	if(TRIM&0x02)EXTI->RTSR|=1<<BITx;	//line BITx上事件上升降沿触发
//}


//外部中断初始化函数
void EXTIX_Falling_Init(void)
{
 	  EXTI_InitTypeDef EXTI_InitStructure;
 	  NVIC_InitTypeDef NVIC_InitStructure;
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//外部中断，需要使能AFIO时钟

    //GPIOA.0	  中断线以及中断初始化配置
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);
	
   	EXTI_InitStructure.EXTI_Line=EXTI_Line0;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // 下降沿触发
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);		//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

  	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//使能按键所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//子优先级1
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器	
}

void EXTIX_Rising_Init(void)
{
 	  EXTI_InitTypeDef EXTI_InitStructure;
 	  NVIC_InitTypeDef NVIC_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//外部中断，需要使能AFIO时钟

    //GPIOA.0	  中断线以及中断初始化配置
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);

   	EXTI_InitStructure.EXTI_Line=EXTI_Line0;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;   //上升沿触发
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);		//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

  	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//使能按键所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;	//抢占优先级2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//子优先级1
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
}

//外部中断服务函数
void EXTI0_IRQHandler(void)  
{
	Get_Counter();
	EXTI_ClearITPendingBit(EXTI_Line0);  //清除EXTI0线路挂起位
}

//自检模块
uint8_t IR_Auto_Check(void)  
{
	if(IR==1) return 1;
	else      return 0;
}

//速度测量函数，返回速度值
double velocity_measurement(void)  
{
	return (double)(DISTANCE/counter*TIMER3_FRQ);  
}


//获得子弹经过光电门计数器计数的次数
void Get_Counter(void)
{
	if(V_Flag.across_flag==0)
	{	  
		TIM_SetCounter(TIM3,0); //TIM3->CNT=0	 
		TIM_Cmd(TIM3, ENABLE);
		V_Flag.across_flag=1;
		EXTIX_Rising_Init(); //上升沿触发
	}
	else
	{
		counter=TIM_GetCounter(TIM3);
		TIM_Cmd(TIM3, DISABLE); 
		V_Flag.across_flag=0;
		V_Flag.shoot_flag=1;
		EXTIX_Falling_Init();  //下降沿触发
	}
}
