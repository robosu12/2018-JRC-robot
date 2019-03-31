#ifndef __LED_H__
#define __LED_H__

void Led_Configuration(void);

#define GREEN_LED_ON()      GPIO_ResetBits(GPIOE, GPIO_Pin_15)
#define GREEN_LED_OFF()     GPIO_SetBits(GPIOE, GPIO_Pin_15)
#define GREEN_LED_TOGGLE()      GPIO_ToggleBits(GPIOE, GPIO_Pin_15)

#define RED_LED_ON()            GPIO_ResetBits(GPIOE, GPIO_Pin_15)
#define RED_LED_OFF()           GPIO_SetBits(GPIOE, GPIO_Pin_15)
#define RED_LED_TOGGLE()        GPIO_ToggleBits(GPIOE, GPIO_Pin_15)


//#define GREEN_LED_ON()          ((void)0)
//#define GREEN_LED_OFF()         ((void)0)
//#define GREEN_LED_TOGGLE()      ((void)0)

//#define RED_LED_ON()            ((void)0)
//#define RED_LED_OFF()           ((void)0)
//#define RED_LED_TOGGLE()        ((void)0)
	
#define BOTH_LED_TOGGLE()\
GPIO_ToggleBits(GPIOE, GPIO_Pin_15);\
GPIO_ToggleBits(GPIOE, GPIO_Pin_15)

#endif
