/*******************************************************************************
 * 文件名  ：rtc.c
 * 描述    ：利用STM32的RTC实现一个简易的电子时钟。
 *           显示格式为 Time: XX : XX : XX(时：分：秒)。         
 * 实验平台：CHD1807-STM32开发板
 * 库版本  ：ST3.5.0
 
**********************************************************************************/
#include "rtc.h"
#include "stdio.h"

/* 秒中断标志，进入秒中断时置1，当时间被刷新之后清0 */
__IO uint32_t TimeDisplay;	

/*
 * 函数名：NVIC_Configuration
 * 描述  ：配置RTC秒中断的主中断优先级为1，次优先级为0
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	
	/* 使能ＲＴＣ中断 */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*
 * 函数名：RTC_Configuration
 * 描述  ：配置RTC
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void RTC_Configuration(void)
{
	/* 使能PWR和BKP时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	
	/* BKP 使能 */
	PWR_BackupAccessCmd(ENABLE);
	
	/* 重设BKP寄存器*/
	BKP_DeInit();
	
	/* 使能 LSE */
	RCC_LSEConfig(RCC_LSE_ON);
	/* 等待LES准备 */
	while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
	{}
	
	/* 选择LES作为RTC时钟 */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
	
	/* 使能RTC时钟*/
	RCC_RTCCLKCmd(ENABLE);
	
	/* 等待RTC同步 */
	RTC_WaitForSynchro();
	
	/* 等待RTC最后的配置完成 */
	RTC_WaitForLastTask();
	
	/* 使能RTC秒 */
	RTC_ITConfig(RTC_IT_SEC, ENABLE);
	

	RTC_WaitForLastTask();
	
	/* 设定ＲＴＣ预标定器 */
	RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
	
	
	RTC_WaitForLastTask();
}


/*
 * 函数名：Time_Regulate
 * 描述  ：返回用户在串口助手中输入的时间值，并将值储存在
 *         RTC 计数寄存器中。
 * 输入  ：无
 * 输出  ：用户在串口助手中输入的时间值，单位为 s
 * 调用  ：内部调用
 */
uint32_t Time_Regulate(void)
{
	uint32_t Tmp_HH = 0xFF, Tmp_MM = 0xFF, Tmp_SS = 0xFF;
	
	printf("\r\n****************设置时间*********************");
	printf("\r\n  请设置小时");
	
	while (Tmp_HH == 0xFF)
	{
		Tmp_HH = USART_Scanf(23);
	}
	printf(":  %d", Tmp_HH);
	printf("\r\n  请设置分钟");
	while (Tmp_MM == 0xFF)
	{
		Tmp_MM = USART_Scanf(59);
	}
	printf(":  %d", Tmp_MM);
	printf("\r\n  请设置秒");
	while (Tmp_SS == 0xFF)
	{
		Tmp_SS = USART_Scanf(59);
	}
	printf(":  %d", Tmp_SS);
	printf("\r\n");
	/* Return the value to store in RTC counter register */
	return((Tmp_HH*3600 + Tmp_MM*60 + Tmp_SS));
}


/*
 * 函数名：Time_Adjust
 * 描述  ：时间调节
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void Time_Adjust(void)
{
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();
	/* Change the current time */
	RTC_SetCounter(Time_Regulate());
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();
}


/*
 * 函数名：Time_Display
 * 描述  ：显示当前时间值
 * 输入  ：-TimeVar RTC计数值，单位为 s
 * 输出  ：无
 * 调用  ：内部调用
 */	
void Time_Display(uint32_t TimeVar)
{
	uint32_t THH = 0, TMM = 0, TSS = 0;
	
	/* Compute  hours */
	THH = TimeVar / 3600;
	/* Compute minutes */
	TMM = (TimeVar % 3600) / 60;
	/* Compute seconds */
	TSS = (TimeVar % 3600) % 60;
	
	printf(" Time: %0.2d:%0.2d:%0.2d\r", THH, TMM, TSS);
	printf("\n\r");
}


/*
 * 函数名：Time_Show
 * 描述  ：在串口助手中显示当前时间值
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */   
void Time_Show(void)
{
	printf("\n\r");
	
	/* Infinite loop */
	while (1)
	{
		/* If 1s has paased */
		if (TimeDisplay == 1)
		{
			/* Display current time */
			Time_Display(RTC_GetCounter());
			TimeDisplay = 0;
		}
	}
}


/*
 * 函数名：USART_Scanf
 * 描述  ：从串口助手获取数值
 * 输入  ：- value 用户在串口助手中输入的数值
 * 输出  ：无
 * 调用  ：内部调用
 */ 
uint8_t USART_Scanf(uint32_t value)
{
	uint32_t index = 0;
	uint32_t tmp[2] = {0, 0};
	
	while (index < 2)
	{
		/* Loop until RXNE = 1 */
		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
		{}
		tmp[index++] = (USART_ReceiveData(USART1));
		// 从串口助手里面输进去的数是ASCII码值
		if ((tmp[index - 1] < 0x30) || (tmp[index - 1] > 0x39))
		{
			printf("\n\rPlease enter valid number between 0 and 9");
			index--;
		}
	}
	/* Calculate the Corresponding value */
	index = (tmp[1] - 0x30) + ((tmp[0] - 0x30) * 10);
	/* Checks */
	if (index > value)
	{
		printf("\n\rPlease enter valid number between 0 and %d", value);
		return 0xFF;
	}
	return index;
}

/*****************************END OF FILE******************************/
