#include "main.h"

/*----GREEN LED----PA6-----'0' is on,'1' is off */
/*----RED LED----PA7-----'0' is on,'1' is off */

void Led_Configuration(void)
{
    GPIO_InitTypeDef gpio;
    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOE,ENABLE);
	
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	
//	gpio.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 |GPIO_Pin_7 |GPIO_Pin_8 | GPIO_Pin_10 | GPIO_Pin_12 | GPIO_Pin_15 ;
//	GPIO_Init(GPIOE,&gpio);  //E12 E15 FORWARD MOTOR ; E4 E5 BACK MOTOR ; E6 7 9 LEFT MOTOR ; E8 10 11 RIGHT MOTOR 
	
	gpio.GPIO_Pin = GPIO_Pin_9 |GPIO_Pin_0; 
	GPIO_Init(GPIOB,&gpio);
	
//	GPIO_SetBits(GPIOE, GPIO_Pin_7);
//	GPIO_SetBits(GPIOE, GPIO_Pin_10);
//	
//	delay_ms(100);  
//	
//	GPIO_ResetBits(GPIOE, GPIO_Pin_7);
//	GPIO_ResetBits(GPIOE, GPIO_Pin_10);

}
