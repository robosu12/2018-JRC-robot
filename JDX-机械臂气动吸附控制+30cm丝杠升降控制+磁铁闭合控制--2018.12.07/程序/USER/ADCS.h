#ifndef _ADCS_H
#define _ADCS_H
#include"stm32f10x.h"
#include "stm32f10x_adc.h"


void ADC1_GPIO_Configuration(void);
void ADC1_RCC_Configuration(void);
void ADC1_Init(void);
void GetADCSConvertedValues(void);

#endif
