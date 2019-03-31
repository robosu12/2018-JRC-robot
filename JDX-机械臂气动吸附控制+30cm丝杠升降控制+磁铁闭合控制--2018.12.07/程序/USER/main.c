
/******************** CHD1807 Team **************************
ROS control : usart1 115200 bps
IO control : A1 A2 A3 A4 A5 A6
2018.09.07

2018.12.02: 磁铁闭合 AA BB 30 打开 AA BB 31 气动吸附打开 AA BB 41 关闭 AA BB 40 丝杠升起 AA BB 51 丝杠下降 AA BB 50 
**********************************************************************************/
#include "stm32f10x.h"
#include "lcd.h" 
#include "systick.h" 
#include "touch.h"	
#include "time_test.h"
#include "distant.h" 
#include "math.h"
#include "receive.h"
#include "GPS.h"
#include "close_GPS.h"
#include "usart1.h"
#include "send.h"
#include "speech.h"
#include "pwm_output.h"
#include "lcd5110.h"
#include"ADCS.h"


__IO uint16_t ADCConvertedValue[10];
__IO uint16_t ADCConvertedValueLocal[10];

extern int LCD_init3_time;//初始化次数检测
extern int LCD_init4_time;
extern uchar volatile rev_ZigBee_stop;
extern volatile unsigned char touch_flag;
extern volatile u32 time;      //当前时刻
extern volatile u8 target_time;//目标翻转时间
extern int time_flag;
void LCD_control(void);//液晶屏显示控制，控制切换界面等等
void Palette_switch(int k);

double speed=0,longitude=108.57165,latitude=34.14101,direct=0,flexible=0;//定义全局变量，速度，经纬度，方向
int luo_pan=0;
extern uchar  rev_GPS_stop; 
GPS_INFO   GPS;
int k=0;
void m_speed(void);
int bm=0;
uchar NS=0,EW=0;
u8 warn_flag=0;
u8 speed_show=1;
u8 count=0;
u8 send_flag=0;
extern u8 time_up;
extern u8 flag_flag;
extern int GPS_TIME;

#define delay_time  1
#define PWM_time  30

int key=20;
int wei=0;
int shu=0,shu_xian=0;
int flag[100]={0};
int flag_chu=0;

int target_dir=180,delta_dir=0;
int r[20]={0};
u32	color_time=0;
double target_longitude=108.57133,target_latitude=34.14133;
float distant_AB=0,distant_lat=0,distant_lon=0;
char flag_ting=0,flag_caiji=0,mo_shi=1;

int avr[10]={0};

char flag_zhangai=0,zhangai=0;

int PWM_left=0,PWM_right=0;

void delay1(int x)
{
int i,j;
for(i=0;i<x;i++)
for(j=0;j<2000;j++);
}

void delay_ad(int x)
{
int i,j;
for(i=0;i<x;i++)
for(j=0;j<10;j++);
}
 

void qian_jin(void)
{
   TIM_SetCompare1(TIM3,600);	  TIM_SetCompare3(TIM3,600);
   TIM_SetCompare2(TIM3,0);     TIM_SetCompare4(TIM3,0);	 	
}

void hou_tui(void)
{
   TIM_SetCompare1(TIM3,0);	  TIM_SetCompare3(TIM3,0);
   TIM_SetCompare2(TIM3,600);   TIM_SetCompare4(TIM3,600);	
}

void zuo_zhuan(void)
{
   TIM_SetCompare1(TIM3,0);	  TIM_SetCompare3(TIM3,900);
   TIM_SetCompare2(TIM3,900);     TIM_SetCompare4(TIM3,0);	
}

void you_zhuan(void)
{
   TIM_SetCompare1(TIM3,900);	  TIM_SetCompare3(TIM3,0);
   TIM_SetCompare2(TIM3,0);     TIM_SetCompare4(TIM3,900);	
}

void ting_zhi(void)
{
   TIM_SetCompare1(TIM3,0);	  TIM_SetCompare3(TIM3,0);
   TIM_SetCompare2(TIM3,0);     TIM_SetCompare4(TIM3,0);	
}

void zhuan(int x,int y)
{

}

void dao_hang(void)
{


}

void control_dir(void)
{
   
 
}

void cai_ji(void)
{
	
}

void AD_convert(void)
{
   
}

void hong_wai(void)
{
	

}

void step_delay_ms(int t)
{
	int i;
	for( i=0;i<t;i++)
	{
		int a=6000; //at 168MHz 42000 is ok
		while(a--);
	}
}

void step_control(int step_count)
{
	if(step_count>0)
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_7 );
		step_delay_ms(1);
		GPIO_ResetBits(GPIOA, GPIO_Pin_7 );
		step_delay_ms(1);
	}
	else
	{
		step_delay_ms(2);
	}
}


int height_motor_control_flag=1;
int step;
int main_count;
/* ****************************************
 * 函数名：main
 * 描述  ：主函数
 * 输入  ：无
 * 输出  ：无
 *******************************************/							
int main(void) 
{	
	SysTick_Init();                     /*systick 初始化*/   
	uart_init(115200);
	
  while (1)   	
  {
    if(height_motor_control_flag==1)
		{
			GPIO_SetBits(GPIOA, GPIO_Pin_6 );
			step_control(step);
			step--;
		}
		else if(height_motor_control_flag==2)
		{
			GPIO_ResetBits(GPIOA, GPIO_Pin_6 );
			step_control(step);
			step--;
		}
		else
		{
			step=0;
		}
		
		if(step<0) step=0;
		
		//step_delay_ms(2);
		if(main_count==1)
		{
			GPIO_SetBits(GPIOB, GPIO_Pin_12 );	 //
			GPIO_SetBits(GPIOB, GPIO_Pin_13 );	 //
			GPIO_SetBits(GPIOB, GPIO_Pin_14 );	 //
		}
		if(main_count==100)
		{
			GPIO_ResetBits(GPIOB, GPIO_Pin_12 );	 //
			GPIO_ResetBits(GPIOB, GPIO_Pin_13 );	 //
			GPIO_ResetBits(GPIOB, GPIO_Pin_14 );	 //
		}
		if(main_count++>200)main_count=0;
		
	  
	}	  
}


/******************* CHD1807 Team *****END OF FILE************/

