#include "FIFO.h"
#include "main.h"

#define LEG_BUF_SIZE 15
#define Front_Leg_Middle 	168 //2048
#define Back_Leg_Middle 	169 //2048

//初始化ADC															   
void  Adc_Init(void)
{    
  GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOA时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC1时钟

  //先初始化ADC1通道5 IO口
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;//PA5 通道5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化  
 
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束	 
 
	
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
  ADC_CommonInit(&ADC_CommonInitStructure);//初始化
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
  ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1 
  ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化
	
 
	ADC_Cmd(ADC1, ENABLE);//开启AD转换器	

}				  
//获得ADC值
//ch: @ref ADC_channels 
//通道值 0~16取值范围为：ADC_Channel_0~ADC_Channel_16
//返回值:转换结果
u16 Get_Adc(u8 ch)   
{
	  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度			    
  
	ADC_SoftwareStartConv(ADC1);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}
//获取通道ch的转换值，取times次,然后平均 
//ch:通道编号
//times:获取次数
//返回值:通道ch的times次转换结果平均值
u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		delay_ms(5);
	}
	return temp_val/times;
} 

extern float forward_leg_angle,back_leg_angle;

void get_leg_angle(void)
{
	static u16 forward_leg_angle_buf[30],back_leg_angle_buf[30];
	static u8 forward_leg_count,back_leg_count;
	u8 i;
	u32 tem_sum=0;
	
	forward_leg_angle = Get_Adc(12) ; //max:AC min:46  change mode angle:5A  leave ground angle: 8A
	back_leg_angle = Get_Adc(13) ;  //max:A8  min:47  change mode angle:5A  leave ground angle: 8E
	
	forward_leg_angle_buf[forward_leg_count++] = forward_leg_angle;
	back_leg_angle_buf[back_leg_count++] = back_leg_angle;
	
	if(forward_leg_count>=LEG_BUF_SIZE) forward_leg_count=0;
	if(back_leg_count>=LEG_BUF_SIZE) back_leg_count=0;
	
	tem_sum=0;
	for(i=0;i<LEG_BUF_SIZE;i++)
	{
		tem_sum += forward_leg_angle_buf[i];
	}
	forward_leg_angle = tem_sum/LEG_BUF_SIZE;
	
	tem_sum=0;
	for(i=0;i<LEG_BUF_SIZE;i++)
	{
		tem_sum += back_leg_angle_buf[i];
	}
	back_leg_angle = tem_sum/LEG_BUF_SIZE;
	
	forward_leg_angle = 1.0*forward_leg_angle/4096*360*3.3/5;
	back_leg_angle = 1.0*back_leg_angle/4096*360*3.3/5;
	
	forward_leg_angle = Front_Leg_Middle - forward_leg_angle;
	back_leg_angle = Back_Leg_Middle - back_leg_angle;
	
//	if((forward_leg_angle>1400&&forward_leg_angle<2800) || (back_leg_angle>1400&&back_leg_angle<2800))
//	{
//		robot_mode = 2;
//	}
//	else
//	if((forward_leg_angle>1000&&forward_leg_angle<1300) && (back_leg_angle>1000&&back_leg_angle<1300))
//	{
//		robot_mode = 1;
//	}
//	else
//	{
//		robot_mode = 2;
//	}
	
//	if((forward_leg_angle>1400) || (back_leg_angle>1400))
//	{
//		robot_mode = 2;
//	}
//	else
//	{
//		robot_mode = 1;
//	}
	
}

