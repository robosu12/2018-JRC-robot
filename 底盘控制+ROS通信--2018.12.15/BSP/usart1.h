#ifndef __USART1_H__
#define __USART1_H__
#include "main.h"
#define PITCH_MAX 21.0f
#define YAW_MAX 720.0f//720.0				//cyq:云台角度的范围
/*
*********************************************************************************************************
*                                               MACROS
*********************************************************************************************************
*/

#define  BSP_USART1_DMA_RX_BUF_LEN               30u                   
 
#define BSP_USART1_RX_BUF_SIZE_IN_FRAMES         (BSP_USART1_RX_BUF_SIZE / RC_FRAME_LENGTH)
#define  RC_FRAME_LENGTH                            18u

void  Adc_Init(void);
u16 Get_Adc(u8 ch);
//返回值:通道ch的times次转换结果平均值
u16 Get_Adc_Average(u8 ch,u8 times);
void get_leg_angle(void);

/*
*********************************************************************************************************
*                                             FUNCTION PROTOTYPES
*********************************************************************************************************
*/
void USART1_IRQHandler(void);
static void USART1_FIFO_Init(void);
void *USART1_GetRxBuf(void);
void USART1_Configuration(uint32_t baud_rate);
void RemoteDataPrcess(uint8_t *pData);

#endif
