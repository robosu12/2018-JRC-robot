/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "sdio_sdcard.h"	
#include "stm32f10x_exti.h"
#include "stm32f10x_tim.h"
#include "lcd.h"
#include "systick.h"
#include "misc.h"
#include "time_test.h"
#include "send.h"
#include "usart1.h"


#define GPS_LEN		80
#define LONLAT_LEN	26
#define uchar unsigned char  

//volatile u32 time=0; //计时变量
volatile unsigned char touch_flag=0;

extern void TimingDelay_Decrement(void);
extern void Delay_us(__IO u32 nTime);
u32 time=0;      //当前时刻
volatile u8 target_time=5;//目标翻转时间
extern int flag_CL;     //背景颜色
extern int flag_LED;     //指示灯当前的颜色
int time_flag=0;

char rev_buf[80];
uchar rev_GPS_start = 0;     //接收GPS开始标志
uchar rev_GPS_stop  = 0;     //接收GPS停止标志
uchar volatile gps_flag = 0;      //GPS处理标志
uchar volatile  GPS_num = 0;  

char rev_lolt[26];//={"@3414.01910,10857.01139"};		   //接收经纬度区，别车位置
uchar volatile rev_ZigBee_start=0;	//接收ZigBee开始标志
uchar volatile rev_ZigBee_stop=0;	//接收ZigBee结束标志
uchar volatile rev_ZigBee_flag=0;		//处理ZigBee数据
uchar volatile snd_ZigBee_start=0;
uchar volatile snd_ZigBee_stop=0;
uchar volatile snd_ZigBee_flag=0;
uchar volatile snd_num=0;
uchar volatile ZigBee_num = 0;
extern int flag,flag2,flag3;
extern char  snd_lolt[23];
extern u8 warn_flag;
int time_count=0;
extern u8 speed_show;
extern int flag_Message;
extern u8 count;
u8 count1=0;
u8 count2=0;
u8 time_up=1;
extern int flag2,flag3,flag,k;
extern u8 send_flag;
extern uchar  ZigBee_Valid;
u8 flag_flag=0;

u8 speech_num=0;
uchar xor=0;
extern uchar send_speech[200];

extern int key;
extern int flag_chu;  
extern int luo_pan;

char i=0;
char rbuff[10]={0};

extern int target_dir;
extern char flag_ting,flag_caiji,mo_shi;

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
extern int height_motor_control_flag;
extern int step;
void grasper_suck_control(unsigned char data)
{
	char count=0;
	static unsigned char flag=0;
	static unsigned char data_buf;
	if(flag==0 && data==0xaa)
	{
		flag=1;
	}
	else if(flag==1 && data==0xbb)
	{
		flag=2;
	}
	else if(flag==2)
	{
		data_buf = data;
		if(data_buf==0x10) 
		{
			
		}
		if(data_buf==0x11)
		{
			
		}
		if(data_buf==0x20) 
		{
			
		}
		if(data_buf==0x21)
		{
			
		}
		if(data_buf==0x30) 
		{

		}
		if(data_buf==0x31)
		{

		}
		if(data_buf==0x50) 
		{
			height_motor_control_flag=1;
			step=12000;
		}
		if(data_buf==0x51)
		{
			height_motor_control_flag=2;
			step=12000;
		}
		if(data_buf==0x20) GPIO_ResetBits(GPIOA, GPIO_Pin_2 );	 //
		if(data_buf==0x21) GPIO_SetBits(GPIOA, GPIO_Pin_2 );	 //
		if(data_buf==0x30) GPIO_ResetBits(GPIOA, GPIO_Pin_3 );	 //
		if(data_buf==0x31) GPIO_SetBits(GPIOA, GPIO_Pin_3 );	 //
		if(data_buf==0x40) GPIO_ResetBits(GPIOA, GPIO_Pin_4 );	 //
		if(data_buf==0x41) GPIO_SetBits(GPIOA, GPIO_Pin_4 );	 //
//		if(data_buf==0x50) GPIO_ResetBits(GPIOA, GPIO_Pin_5 );	 //
//		if(data_buf==0x51) GPIO_SetBits(GPIOA, GPIO_Pin_5 );	 //
//		if(data_buf==0x60) GPIO_ResetBits(GPIOA, GPIO_Pin_6 );	 //
//		if(data_buf==0x61) GPIO_SetBits(GPIOA, GPIO_Pin_6 );	 //
	}
	else
	{
		flag=0;
	}
	
}

void USART1_IRQHandler(void)
{
	char ch=0;
	if (USART_GetITStatus(USART1,USART_IT_RXNE) != RESET)    //确认发生接收中断 
  {    	       
     USART_ClearITPendingBit(USART1, USART_IT_RXNE);        //清除中断标志 
		
		grasper_suck_control(USART_ReceiveData(USART1));

	}

}

void USART2_IRQHandler(void)
{
	 char t;
	char j=0;
	char f=0;                                                       //定义字符变量 

   if (USART_GetITStatus(USART2,USART_IT_RXNE) != RESET)    //确认发生接收中断 
   {    	       
    USART_ClearITPendingBit(USART2, USART_IT_RXNE);        //清除中断标志 

    t=USART_ReceiveData(USART2);
	rbuff[i++]=t;
	if(t=='*')
	{
	 i=0;
	 f=1;
	}

	if(f==1)
	{
	  f=0;
   	  luo_pan=rbuff[0]*100+rbuff[1]*10+rbuff[2]-10;

	  if(luo_pan<0) luo_pan=luo_pan+360;
	  rbuff[0]=rbuff[0]+0x30;
	  rbuff[1]=rbuff[1]+0x30;
	  rbuff[2]=rbuff[2]+0x30;

	for(j=0;rbuff[j]!='*';j++)
	{
	  USART_SendData(USART2,rbuff[j]);//把接收到的数据发送出
 	  while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}//等待发送结束
	}
	
	 USART_SendData(USART2,rbuff[j]);//把接收到的数据发送出
 	  while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}//等待发送结束	

	for(j=0;rbuff[j]!='*';j++)
	{
	   rbuff[j]=0;
	}
	   rbuff[j]=0;

	}

	}
	 

}


void USART3_IRQHandler(void)
{
	 char t;                                                      //定义字符变量 

   if (USART_GetITStatus(USART3,USART_IT_RXNE) != RESET)    //确认发生接收中断 
   {    	       
    USART_ClearITPendingBit(USART3, USART_IT_RXNE);        //清除中断标志 

    t=USART_ReceiveData(USART3);

    if(t==1)	//前进
    {
	 target_dir=180;//qian_jin();
	 SYN_FrameInfo(0,"向北移动");
	 qian_jin();
	 flag_ting=0;
    }

    if(t==2)	//后退
    {
	  target_dir=0;//hou_tui();
	  SYN_FrameInfo(0,"向南移动");
	  hou_tui();
	  flag_ting=0;
    }

    if(t==3)  //向左
    {
	 target_dir=90;//zuo_zhuan();
	 SYN_FrameInfo(0,"向西移动");
	 zuo_zhuan();
	 flag_ting=0;
    }

    if(t==4)	//向右
    {
	 target_dir=270;//you_zhuan();
	 SYN_FrameInfo(0,"向东移动");
	 you_zhuan();
	 flag_ting=0;
    }

    if(t==5)	 //停止
    {
	 SYN_FrameInfo(0,"停止前进");
	 ting_zhi();
	 flag_ting=1;
    }

    if(t==7)	 //前进
    {
	  flag_caiji=1;
    }

    if(t==8)	 //前进
    {
	  flag_caiji=0;
    }

    if(t==9)	 //前进
    {
	  mo_shi=1;
	  SYN_FrameInfo(0,"定位导航模式");
    }

    if(t==10)	 //前进
    {
	  mo_shi=2;
	  SYN_FrameInfo(0,"方向导航模式");
    }

    if(t==11)	 //前进
    {
	  mo_shi=3;
	  SYN_FrameInfo(0,"红外避障模式");
    }

   }

}

void TIM2_IRQHandler(void)
{ 
	if ( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET ) 
	{	
	  time++;

	  if(time>2)
	  {
	   time=0;
	   GPIO_WriteBit(GPIOD,GPIO_Pin_13,1-GPIO_ReadOutputDataBit(GPIOD,GPIO_Pin_13));
	  }
	   							
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);
		 
	}
}

 
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	TimingDelay_Decrement();  /* 系统滴答中断处理函数 */
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 
/*
 * 函数名：SDIO_IRQHandler
 * 描述  ：在SDIO_ITConfig(）这个函数开启了sdio中断	，
 			数据传输结束时产生中断
 * 输入  ：无		 
 * 输出  ：无
 */
void SDIO_IRQHandler(void) 
{
  /* Process All SDIO Interrupt Sources */
  SD_ProcessIRQSrc();
}

void EXTI9_5_IRQHandler(void)
{ 

 
  if(EXTI_GetITStatus(EXTI_Line6) != RESET)
  {
     GPIO_ResetBits(GPIOB, GPIO_Pin_5);   								 	 	   			    

			touch_flag=1;

    EXTI_ClearITPendingBit(EXTI_Line6);
  }
}
/**
  * @brief  This function handles TIM3 interrupt request.
  * @param  None
  * @retval : None
  */
void TIM3_IRQHandler(void)
{
	
	if ( TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET ) 
	{	 
	   
		 TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);  	
	
   } 
 }

 
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
