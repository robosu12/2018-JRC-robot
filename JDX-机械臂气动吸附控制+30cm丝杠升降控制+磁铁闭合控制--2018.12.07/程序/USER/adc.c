#include "adc.h"
#include "math.h"

/**************角度相关变量*************************/
//int g_nLeftMotorOut,g_nRightMotorOut;	
//int count0=0,count1=0,count2=0,count3=0,count4=0,count11=0,flag2=0,flag1=0;
float g_nCarGyroVal=0,anglez=0, angle_kalman,angle_dot_kalman;
float gyroscope_rate=0;  
int x,y,z;
float  Ax,Ay,Az,z_1;
//float  Axyz,Ayxz,Azxy,angle_YZ; 
float  AD_gyro=0,Gyro_b =0,angle_kalman_last;
int   Gyro_zero=1533;// 陀螺仪零点设置
float ZAdMax=1560.0,ZAdMin=460.0;//加速度计最大值和最小值	 
 /***************************************************
*参数:z_max=2830 z_min=800
*返回值:无
*	（z_max + z_min）/2=1815;
*	（z_max -z_min）/2=1000;
****************************************************/
void acc_z(void)  
{
	z=Get_Adc(ADC_CH1);
	AD_gyro=Get_Adc(ADC_CH2);
   	Gyro_b=Gyro_zero-AD_gyro;
	 Gyro_b=-Gyro_b;
	gyroscope_rate=Gyro_b*0.3802;	//计算出角速度
	Az=(z-(ZAdMax+ZAdMin)/2.0)/((ZAdMax-ZAdMin)/2.0);         //1185   635.0
    if(Az>1.0)Az=1.0;
    if(Az<-1.0)Az=-1.0;
   	anglez=asinf(Az)/3.1416*180;	//计算角度值 
	anglez=-anglez; 
} 
		   
//初始化ADC
//这里我们仅以规则通道为例
//我们默认将开启通道0~3																	   
void  Adc_Init(void)
{    
	//先初始化IO口
 	RCC->APB2ENR|=1<<2;    //使能PORTA口时钟 
	GPIOC->CRL&=0XFFFF0000;//PA0 1 2 3 anolog输入
	//通道10/11设置			 
	RCC->APB2ENR|=1<<9;    //ADC1时钟使能	  
	RCC->APB2RSTR|=1<<9;   //ADC1复位
	RCC->APB2RSTR&=~(1<<9);//复位结束	    
	RCC->CFGR&=~(3<<14);   //分频因子清零	
	//SYSCLK/DIV2=12M ADC时钟设置为12M,ADC最大时钟不能超过14M!
	//否则将导致ADC准确度下降! 
	RCC->CFGR|=2<<14;      	 

	ADC1->CR1&=0XF0FFFF;   //工作模式清零
	ADC1->CR1|=0<<16;      //独立工作模式  
	ADC1->CR1&=~(1<<8);    //非扫描模式	  
	ADC1->CR2&=~(1<<1);    //单次转换模式
	ADC1->CR2&=~(7<<17);	   
	ADC1->CR2|=7<<17;	   //软件控制转换  
	ADC1->CR2|=1<<20;      //使用用外部触发(SWSTART)!!!	必须使用一个事件来触发
	ADC1->CR2&=~(1<<11);   //右对齐	 
	ADC1->SQR1&=~(0XF<<20);
	ADC1->SQR1&=0<<20;     //1个转换在规则序列中 也就是只转换规则序列1 			   
	//设置通道0~3的采样时间
	ADC1->SMPR2&=0XFFFFF000;//通道0,1,2,3采样时间清空	  
	ADC1->SMPR2|=7<<9;      //通道3  239.5周期,提高采样时间可以提高精确度	 
	ADC1->SMPR2|=7<<6;      //通道2  239.5周期,提高采样时间可以提高精确度	 
	ADC1->SMPR2|=7<<3;      //通道1  239.5周期,提高采样时间可以提高精确度	 
	ADC1->SMPR2|=7<<0;      //通道0  239.5周期,提高采样时间可以提高精确度	 

	ADC1->CR2|=1<<0;	    //开启AD转换器	 
	ADC1->CR2|=1<<3;        //使能复位校准  
	while(ADC1->CR2&1<<3);  //等待校准结束 			 
    //该位由软件设置并由硬件清除。在校准寄存器被初始化后该位将被清除。 		 
	ADC1->CR2|=1<<2;        //开启AD校准	   
	while(ADC1->CR2&1<<2);  //等待校准结束
	//该位由软件设置以开始校准，并在校准结束时由硬件清除  
}				  
//获得ADC值
//ch:通道值 0~3
u16 Get_Adc(u8 ch)   
{
	//设置转换序列	  		 
	ADC1->SQR3&=0XFFFFFFE0;//规则序列1 通道ch
	ADC1->SQR3|=ch;		  			    
	ADC1->CR2|=1<<22;       //启动规则转换通道 
	while(!(ADC1->SR&1<<1));//等待转换结束	 	   
	return ADC1->DR;		//返回adc值	
}




























