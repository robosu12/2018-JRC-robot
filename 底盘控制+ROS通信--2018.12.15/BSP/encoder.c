#include "main.h"

//encoder.INPUT_B---PB4(TIM3_CH1)
//encoder.INPUT_A---PB5(TIM3_CH2)

void Quad_Encoder_Configuration(void)
{
    GPIO_InitTypeDef gpio;
	
	  TIM_ICInitTypeDef TIM_ICInitStruct_1;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	  gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
	  gpio.GPIO_Mode = GPIO_Mode_AF;
	  gpio.GPIO_PuPd = GPIO_PuPd_UP;
	  GPIO_Init(GPIOB,&gpio);
	
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_TIM3);
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_TIM3);
	
	  TIM_ICStructInit(&TIM_ICInitStruct_1);
	  TIM_ICInitStruct_1.TIM_ICFilter = 0X05;
  	TIM_ICInit(TIM3, &TIM_ICInitStruct_1);
	
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
    TIM_Cmd(TIM3,ENABLE);	 
	
	/////////////////////////////////////////////////////
	
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	  gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
	  gpio.GPIO_Mode = GPIO_Mode_AF;
	  gpio.GPIO_PuPd = GPIO_PuPd_UP;
	  GPIO_Init(GPIOB,&gpio);
	
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
    TIM_Cmd(TIM4,ENABLE);	

    TIM3->CNT = 0xffff;
		TIM4->CNT = 0xffff;
}

void Encoder_Start(void)
{
    TIM3->CNT = 0x7fff;
}

static char Encoder_Dir = 0;
  
int32_t GetQuadEncoderDiff(void)
{
    int32_t cnt = 0;    
    cnt = (TIM3->CNT)-0x7fff;
    TIM3->CNT = 0x7fff;    
    if(Encoder_Dir == 1)
	{
		return cnt;	
	}
	else
	{
		return -cnt;            
	}
}
