#include "stm32f4xx.h"
#include "gun.h"
/*-LEFT---(PB3---TIM2_CH2)--*/
/*-RIGHT--(PA15--TIM2_CH1)--*/

void PWM_Configuration(void)
{
    GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA ,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);   //PCLK1=42MHz,TIM5 clk =84MHz
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);   //PCLK1=42MHz,TIM2 clk =84MHz

    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA,&gpio);

    GPIO_PinAFConfig(GPIOA,GPIO_PinSource0, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5); 
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM9);	
    /* TIM5 */
    tim.TIM_Prescaler = 84-1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 2500;   //2.5ms
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM5,&tim);
		
    /* TIM9 */
		tim.TIM_Prescaler = 168-1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 1000;   //1ms,1KHz
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM9,&tim);
		
    oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputState_Disable;
    oc.TIM_Pulse = 1000;
    oc.TIM_OCPolarity = TIM_OCPolarity_Low;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Set;
    TIM_OC1Init(TIM5,&oc);
    TIM_OC2Init(TIM5,&oc);
		oc.TIM_Pulse = 0;
		TIM_OC1Init(TIM9,&oc);
    
    TIM_OC1PreloadConfig(TIM5,TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM5,TIM_OCPreload_Enable);
		TIM_OC1PreloadConfig(TIM9,TIM_OCPreload_Enable);
           
    TIM_ARRPreloadConfig(TIM5,ENABLE);
    TIM_ARRPreloadConfig(TIM9,ENABLE);
		
    TIM_Cmd(TIM5,ENABLE);
		TIM_Cmd(TIM9,ENABLE);
}

#define INIT_DUTY 0 //u16(1000/0.25)
#define ACCURACY 1000 //u16(2500/0.25) //accuracy
#define PWM_RADIO 4//(8000 - 4000)/1000.0

void PWM_Out_Init(int hz)//400hz
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	uint16_t PrescalerValue = 0;
	u32 hz_set = ACCURACY*hz;

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_OCStructInit(&TIM_OCInitStructure);
	
	//hz_set = LIMIT (hz_set,1,84000000);
	
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

////////////////////////////// TIM5 / 50HZ ///////////////////////////////////////////////
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1|  GPIO_Pin_2| GPIO_Pin_3  ; //  |  GPIO_Pin_2| GPIO_Pin_3
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(GPIOA, &GPIO_InitStructure); 

//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5); 
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);
//	
//	// Compute the prescaler value 
//  PrescalerValue = (uint16_t) ( ( SystemCoreClock /2 ) / hz_set ) - 1;
//  // Time base configuration 
//  TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
//  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
//  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);


//  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

////   PWM1 Mode configuration: Channel1 
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC1Init(TIM5, &TIM_OCInitStructure);
//  TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);

////   PWM1 Mode configuration: Channel2 
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC2Init(TIM5, &TIM_OCInitStructure);
//  TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);

//  // PWM1 Mode configuration: Channel3 
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC3Init(TIM5, &TIM_OCInitStructure);
//  TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);

//  // PWM1 Mode configuration: Channel4 
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC4Init(TIM5, &TIM_OCInitStructure);
//  TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
//	
//  TIM_ARRPreloadConfig(TIM5, ENABLE);
//  TIM_Cmd(TIM5, ENABLE);
/////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////// TIM1 ///////////////////////////////////////////////////////////
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOE, &GPIO_InitStructure); 

  GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1); 
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
	
	/* Compute the prescaler value */
  PrescalerValue = (uint16_t) ( ( SystemCoreClock ) / hz_set ) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);


  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
  //TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
  //TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
  //TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC4Init(TIM1, &TIM_OCInitStructure);
  //TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
  TIM_ARRPreloadConfig(TIM1, ENABLE);
  TIM_Cmd(TIM1, ENABLE);	
	////////////////////////////////////////////////////////////////////////////////////
	
  ///////////////////////////////// TIM8 ///////////////////////////////////////////////////
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 |GPIO_Pin_7 |GPIO_Pin_8 | GPIO_Pin_9; 
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(GPIOC, &GPIO_InitStructure); 

//  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
//  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
//	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
//  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);
//	
//	/* Compute the prescaler value */
//  PrescalerValue = (uint16_t) ( ( SystemCoreClock ) / hz_set ) - 1;
//  /* Time base configuration */
//  TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
//  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
//  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
//  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);


//  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
//  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
//	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
//	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

//  /* PWM1 Mode configuration: Channel3 */
//  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC1Init(TIM8, &TIM_OCInitStructure);
//  //TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

//  /* PWM1 Mode configuration: Channel4 */
//  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC2Init(TIM8, &TIM_OCInitStructure);
//  //TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
//	
//	TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC3Init(TIM8, &TIM_OCInitStructure);
//  //TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
//	
//	TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC4Init(TIM8, &TIM_OCInitStructure);
//  //TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
//	
//	TIM_CtrlPWMOutputs(TIM8, ENABLE);
//  TIM_ARRPreloadConfig(TIM8, ENABLE);
//  TIM_Cmd(TIM8, ENABLE);
	//////////////////////////////////////////////////////////////////////////////////
	
//	TIM5->CCR1 = 0;
//	TIM5->CCR2 = 0;
//	TIM5->CCR3 = 0;
//	TIM5->CCR4 = 0;
	
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = 0;
	
//	TIM8->CCR1 = 0;
//	TIM8->CCR2 = 0;
//	TIM8->CCR3 = 0;
//	TIM8->CCR4 = 0;

}
