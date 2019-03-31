#ifndef TIME_TEST_H
#define TIME_TEST_H

#include "stm32f10x.h"

#define START_TIME  time=0; RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);TIM_Cmd(TIM2, ENABLE)
#define STOP_TIME  TIM_Cmd(TIM2, DISABLE);RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE)

#define START_TIME2  TIM_Cmd(TIM2, ENABLE)
#define STOP_TIME2  TIM_Cmd(TIM2, DISABLE)

void GPIOD_Config(void);
void TIM3_NVIC_Configuration(void);
void TIM3_Configuration(void);
void TIM3_IT_Config(void);

void TIM2_IT_Config(void);
void GPIOA_Config(void);
void TIM2_NVIC_Configuration(void);
void TIM2_Configuration(void);
#endif	/* TIME_TEST_H */
