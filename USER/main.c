#include "main.h"

 int main(void)
 {	
	double v;
	char str[20];	
	 
	BSP_INIT();
	
	while(!IR_Auto_Check()) //自检错误进入循环
	{
		printf("surprise,something wrong ^_^");
		OLED_ShowString(0,0,"surprise,something wrong ^_^",16);
	}
	
	//运行正常
	printf("well done ^_^\r\n");
	OLED_Clear(); 
	OLED_ShowString(0,0,"well done ^_^",16);

	while(1)
	{
	LED0=!LED0;
	if(V_Flag.shoot_flag==1)
	{
		v=velocity_measurement();
		
		sprintf(str,"v:%5.2fm/s",v);
		OLED_ShowString(0,3,str,16);
		
		printf("velocity:%f m/s r\n",v);
		V_Flag.shoot_flag=0;
	}
//	OLED_Clear(); 
		delay_ms(1);
	}
	
}
