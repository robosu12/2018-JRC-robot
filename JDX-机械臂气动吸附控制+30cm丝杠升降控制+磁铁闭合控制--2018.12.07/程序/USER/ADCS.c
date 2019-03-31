//多路ADC数据采集C文件
#include"ADCS.h"
extern __IO uint16_t ADCConvertedValue[10];
//=========================================================================================
//变量定义


//ADC 外设的数据寄存器
#define ADC1_DR_Address    ((uint32_t)0x4001244C)
//ADC_DR(ADC规则数据寄存器),偏移量=0x4c  ADC1(0x40012400-0x400127ff)
//so ADC1_DR_Address=0x40012400+0x4c


//==========================================================================================
//ADC1_GPIO配置
void ADC1_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  //CH_10->PC0	CH_11->PC1	CH_12->PC2	CH_13->PC3
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	//adc模式必须是模拟输入
  GPIO_Init(GPIOC, &GPIO_InitStructure);//执行上面的操作

  //CH_0->PA0	CH_1->PA1	CH_2->PA2	CH_3->PA3
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	//adc模式必须是模拟输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//执行上面的操作
  //注意，内部温度对应 16通道，无引脚，只需开启adc时钟即可。
  //内部参考电压，对应17 通道。无引脚。只需开启时钟
}

//============================================================================================
//外设ADC，DMA时钟开启
 void ADC1_RCC_Configuration(void)
{
  /* Enable ADC1 and GPIOC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOA, ENABLE);
}

//===============================================================================================
//ADC1 
void ADC1_Init(void)
{
ADC_InitTypeDef ADC_InitStructure;

  /* ADC1 configuration ------------------------------------------------------*/
  //ADC独立模式	 相对于双重模式
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  //扫描模式用于多通道采集
  //而这次虽然也是多通道，单是采用非DMA模式，避免扫描模式因读取数据不及时而造成下一个通道的
  //数据覆盖本组数据，所以采用单次转换模式，即：采集完成本组后，并且数据被读走，才开始下一个
  //通道的工作
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  //开启连续转换模式   当转换完本组（可能是一个）继续重新开始执行
  //相对于单次模式：转换一次后就结束
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  //不使用外部触发转换
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  //采集数据右对齐
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  //转换组的通道数目
  ADC_InitStructure.ADC_NbrOfChannel =1;
  //执行
  ADC_Init(ADC1, &ADC_InitStructure);
  
  //配置ADC时钟，为PCLK2的8分频，即9Hz
  RCC_ADCCLKConfig(RCC_PCLK2_Div8);
  /* ADC1 regular channel11 configuration */ 
  //配置ADC1的通道11为55.5个采样周期
  //默认组，adc1 ，通道11，排序为1,55.5周期
  
  
  

  //----------------------使能温度传感器----------------------------
  ADC_TempSensorVrefintCmd(ENABLE);

  
   ADC_Cmd(ADC1, ENABLE);
  // ADC_SoftwareStartConvCmd(ADC1, DISABLE);
  /* Enable ADC1 reset calibration register */ 
  //使能ADC1的复位校准寄存器  
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  //等待校准完成
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibration */
  //使能ADC1的开始校准寄存器
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  //等待完成
  while(ADC_GetCalibrationStatus(ADC1));
  

}
//===============================================================================================
//说明：3个通道的数据，分别按单次转换，完成，结果存放在全局变量ADCConvertedValue【】中，在主函数中显示
//													  
void GetADCSConvertedValues()
{
	uint8_t i;
	for(i=0;i<3;i++)
	{
		switch(i)
		{
		
		case 0:	 //通道10（外部可变电阻电压），规则组序号1，。。。
		ADC_RegularChannelConfig(ADC1, ADC_Channel_10,1, ADC_SampleTime_239Cycles5);break;
		case 1:	 //通道11（外部可变电阻电压），规则组序号1，。。。
		ADC_RegularChannelConfig(ADC1, ADC_Channel_11,1, ADC_SampleTime_239Cycles5);break;
		case 2:	 //通道12（外部可变电阻电压），规则组序号1，。。。
		ADC_RegularChannelConfig(ADC1, ADC_Channel_12,1, ADC_SampleTime_239Cycles5);break;
		
		
		}
		
  		ADC_Cmd(ADC1, ENABLE); //使能ADC1
  		ADC_SoftwareStartConvCmd(ADC1, ENABLE);	//启动上面设置好的一个通道，进行转换	
		while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);//等待EOC置位
		ADCConvertedValue[i]=ADC_GetConversionValue (ADC1);	//把数据寄存器的值读走

		ADC_ClearFlag(ADC1, ADC_FLAG_EOC);	   //清除EOC，DMA时读数据，硬件自动清除
        ADC_SoftwareStartConvCmd(ADC1, DISABLE);
        ADC_Cmd(ADC1, DISABLE);
	}
}
