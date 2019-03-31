/******************** CHD1807 Team **************************
 * 文件名  ：Touch.c
 * 描述    ：LCD触摸应用函数库         
 * 实验平台：CHD1807-STM32开发板
 * 库版本  ：ST3.5.0
**********************************************************************************/
#include "touch.h"
#include "systick.h"
#include "lcd.h"
#include "stm32f10x_spi.h"
#include "misc.h"
#include "stm32f10x_exti.h"
#include "lcd_botton.h"
//#include "sd_fs_app.h"
//#include "Sd_bmp.h"
extern volatile unsigned char touch_flag;

Parameter touch_para ;
Coordinate  display ;



/* DisplaySample LCD坐标上对应的ads7843采样AD值 如：LCD 坐标45,45 应该的X Y采样ADC分别为3388,920 */	
Coordinate ScreenSample[4];



/* LCD上的坐标 */
Coordinate DisplaySample[4] =   
{
    { 45, 45 },
    { 45, 270},
    { 190,190},
    {210,280}
} ;



/* 差值门限 */
#define THRESHOLD 2   


int LCD_init_time=0;//初始化次数检测
int LCD_init3_time=0;//初始化次数检测
int LCD_init4_time=0;
/******************************************************
* 函数名：Touch_spi_init
* 描述  ：2046 Touch SPI 初始化
* 输入  : 无
* 输出  ：无
* 举例  ：无
* 注意  ：无
*********************************************************/    
static void Touch_spi_init(void) 
{ 
    SPI_InitTypeDef  SPI_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    
    /* DISABLE SPI1 */ 
    SPI_Cmd(SPI1, DISABLE); 
    
    /* SPI1 配置 --*/ 
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master; 
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; 
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; 
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; 
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; 
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; 
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; 
    SPI_InitStructure.SPI_CRCPolynomial = 7; 
    SPI_Init(SPI1, &SPI_InitStructure); 
    
		/* Enable SPI1 */ 
    SPI_Cmd(SPI1, ENABLE); 
} 





/******************************************************
* 函数名：Touch_init
* 描述  ：2046 Touch 端口初始化
* 输入  : 无
* 输出  ：无
* 举例  ：无
* 注意  ：无
*********************************************************/    
void Touch_init(void) 
{ 
    EXTI_InitTypeDef EXTI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
    
    /* 配置SPI1管脚: SCK--PA5, MISO--PA6 and MOSI--PA7 ---------------------------------*/ 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5  | GPIO_Pin_6 | GPIO_Pin_7; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;       
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
    /* 2046CS --PB7 */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* 2046 INT_IRQ --PB6*/
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
    TOUCH_NVIC_Configuration();
    EXTI_InitStructure.EXTI_Line = EXTI_Line6;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource6);
    
    CS_2046(1); 
    Touch_spi_init(); 
} 





/******************************************************
* 函数名：TOUCH_NVIC_Configuration
* 描述  ：2046 Touch INT 嵌套中断配置
* 输入  : 无
* 输出  ：无
* 举例  ：无
* 注意  ：留意一下优先级
*********************************************************/    
void TOUCH_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;


    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
 

    /*使能EXTI9_5 中断 */		      
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn  ;    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
}




/******************************************************
* 函数名：DelayUS
* 描述  ：粗略延时cnt 微妙
* 输入  : cnt
* 输出  ：无
* 举例  ：无
* 注意  ：该函数创建的目的是局部使用,从而不必特意去使用定时器来延时
*********************************************************/    
static void DelayUS(vu32 cnt)
{
    uint16_t i;
    for(i = 0;i<cnt;i++)
    {
        uint8_t us = 12; /* 设置值为12，大约延1微秒 */    
        while (us--)     /* 延1微秒	*/
        {
            ;   
        }
    }
}






/******************************************************
* 函数名：WR_CMD
* 描述  ：向 Touch 2046写数据
* 输入  : cmd 		--需要写入的数据
* 输出  ：无
* 举例  ：无
* 注意  ：内部使用
*********************************************************/    
static void WR_CMD (uint8_t cmd)  
{ 
    /* Wait for SPI1 Tx buffer empty */ 
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); 
    /* Send SPI1 data */ 
    SPI_I2S_SendData(SPI1,cmd); 
    /* Wait for SPI1 data reception */ 
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 
    /* Read SPI1 received data */ 
    SPI_I2S_ReceiveData(SPI1); 
} 





/******************************************************
* 函数名：Read_touch_ad
* 描述  ：通过SPI方式读取 2046所获得的触摸AD转换值
* 输入  : 无
* 输出  ：无
* 举例  ：无
* 注意  ：内部使用
*********************************************************/    
static int Read_touch_ad(void)  
{ 
    unsigned short buf,temp; 
    /*  等待数据寄存器空 */  
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); 

    /* 通过SPI1发送数据 */  
    SPI_I2S_SendData(SPI1,0x0000); 

    /* 等待接收到一个字节的数据 */ 
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 

    /* 读取SPI1接收的数据 */		 
    temp=SPI_I2S_ReceiveData(SPI1); 

    buf=temp<<8; 
    DelayUS(1);

		/*  等待数据寄存器空 */  
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); 

    /* 通过SPI1发送数据 */ 
    SPI_I2S_SendData(SPI1,0x0000); 

    /* 等待接收到一个字节的数据 */ 
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 
 
    /* 读取SPI1接收的数据 */ 
    temp=SPI_I2S_ReceiveData(SPI1); 
 
    buf |= temp; 
    buf>>=3; 
    buf&=0xfff; 
    return buf; 

} 







/******************************************************
* 函数名：Read_X
* 描述  ：通过SPI方式读取 2046所获得的触摸 通道X+ AD转换值
* 输入  : 无
* 输出  ：X+通道AD转换值
* 举例  ：无
* 注意  ：无
*********************************************************/    
int Read_X(void)  
{  
    int i; 
    CS_2046(0); 
    DelayUS(1); 
    WR_CMD(CHX); 
    DelayUS(1); 
    i=Read_touch_ad(); 
    CS_2046(1); 
    return i;    
} 






/******************************************************
* 函数名：Read_Y
* 描述  ：通过SPI方式读取 2046所获得的触摸 通道Y+ AD转换值
* 输入  : 无
* 输出  ：Y+通道AD转换值
* 举例  ：无
* 注意  ：无
*********************************************************/    
int Read_Y(void)  
{  
    int i; 
    CS_2046(0); 
    DelayUS(1); 
    WR_CMD(CHY); 
    DelayUS(1); 
    i=Read_touch_ad(); 
    CS_2046(1); 
    return i;     
} 






/******************************************************
* 函数名：Touch_GetAdXY
* 描述  ：SPI方式 读取2046 通道X+ 通道Y+的ADC值
* 输入  : 无
* 输出  ：通道X+ 通道Y+的ADC值
* 举例  ：无
* 注意  ：无
*********************************************************/    
void Touch_GetAdXY(int *x,int *y)  
{ 
    int adx,ady; 
    adx=Read_X(); 
    DelayUS(1); 
    ady=Read_Y(); 
    *x=adx; 
    *y=ady; 
} 






/******************************************************
* 函数名：Palette_draw_point
* 描述  ：在LCD指定位置画一个大点(包含四个小点)
* 输入  : Xpos		--X方向位置
Ypos		--Y方向位置
* 输出  ：无
* 举例  ：Palette_draw_point(100,100);
* 注意  ：该函数是 "触摸画板应用实例" 专用函数
*********************************************************/    
/*void Palette_draw_point(uint16_t Xpos,uint16_t Ypos)
{
    static u16 Pen_color=0;
		u16 Tmp_xpos = 240-Xpos;
    if(Ypos<40)
    {
        if(Tmp_xpos>30)
            Pen_color =(Tmp_xpos<60)?GREEN:(Tmp_xpos<90)?BLUE:(Tmp_xpos<120)?BRED:(Tmp_xpos<150)?GRED:(Tmp_xpos<180)?GBLUE:(Tmp_xpos<210)?BLACK:(Tmp_xpos<240)?RED:BLUE;
        else
        {
            LCD_Rectangle(40,0,280,240,WHITE);
            return;
        }
    }
    else
    {
				LCD_ColorPoint(Ypos,240-Xpos,Pen_color);
				LCD_ColorPoint(Ypos+1,240-Xpos,Pen_color);
				LCD_ColorPoint(Ypos,(240-Xpos)+1,Pen_color);
				LCD_ColorPoint(Ypos+1,(240-Xpos)+1,Pen_color);
    }	
}	
*/




/******************************************************
* 函数名：DrawCross
* 描述  ：在LCD指定位置画十字
* 输入  : Xpos		--X方向位置
Ypos		--Y方向位置
* 输出  ：无
* 举例  ：DrawCross(100,100);
* 注意  ：触摸校正专用
*********************************************************/    
void DrawCross(uint16_t Xpos,uint16_t Ypos)
{
    u8 b;    
    for(b=0; b<10; b++)
    {
        LCD_Point(Ypos-b,240-Xpos);
        LCD_Point(Ypos+b,240-Xpos);
        LCD_Point(Ypos,240-(Xpos+b));
        LCD_Point(Ypos,240-(Xpos-b));
    }
    
    
}	



/******************************************************
* 函数名：Read_2046
* 描述  ：得到滤波之后的X Y
* 输入  : 无
* 输出  ：Coordinate结构体地址
* 举例  ：无
* 注意  ：速度相对比较慢
*********************************************************/    
Coordinate *Read_2046(void)
{
    static Coordinate  screen;
    int m0,m1,m2,TP_X[1],TP_Y[1],temp[3];
    uint8_t count=0;
		
		/* 坐标X和Y进行9次采样*/
    int buffer[2][9]={{0},{0}};  	
    do
    {		   
        Touch_GetAdXY(TP_X,TP_Y);  
        buffer[0][count]=TP_X[0];  
        buffer[1][count]=TP_Y[0];
        count++; 
				 
    }	/*用户点击触摸屏时即TP_INT_IN信号为低 并且 count<9*/
    while(!INT_IN_2046&& count<9);  
    
		
		/*如果触笔弹起*/
		if(INT_IN_2046 ==1)
    {
        /*中断标志复位*/
				touch_flag =0;						
    }

		/* 如果成功采样9次,进行滤波 */ 
    if(count==9)   								
    {  
        /* 为减少运算量,分别分3组取平均值 */
        temp[0]=(buffer[0][0]+buffer[0][1]+buffer[0][2])/3;
        temp[1]=(buffer[0][3]+buffer[0][4]+buffer[0][5])/3;
        temp[2]=(buffer[0][6]+buffer[0][7]+buffer[0][8])/3;
        
        /* 计算3组数据的差值 */
        m0=temp[0]-temp[1];
        m1=temp[1]-temp[2];
        m2=temp[2]-temp[0];
        
        /* 对上述差值取绝对值 */
        m0=m0>0?m0:(-m0);
        m1=m1>0?m1:(-m1);
        m2=m2>0?m2:(-m2);
        
        /* 判断绝对差值是否都超过差值门限，如果这3个绝对差值都超过门限值，则判定这次采样点为野点,抛弃采样点，差值门限取为2 */
        if( m0>THRESHOLD  &&  m1>THRESHOLD  &&  m2>THRESHOLD ) 
            return 0;
        
        /* 计算它们的平均值，同时赋值给screen */ 
        if(m0<m1)
        {
            if(m2<m0) 
                screen.x=(temp[0]+temp[2])/2;
            else 
                screen.x=(temp[0]+temp[1])/2;	
        }
        else if(m2<m1) 
            screen.x=(temp[0]+temp[2])/2;
        else 
            screen.x=(temp[1]+temp[2])/2;
        
        /* 同上 计算Y的平均值 */
        temp[0]=(buffer[1][0]+buffer[1][1]+buffer[1][2])/3;
        temp[1]=(buffer[1][3]+buffer[1][4]+buffer[1][5])/3;
        temp[2]=(buffer[1][6]+buffer[1][7]+buffer[1][8])/3;
        m0=temp[0]-temp[1];
        m1=temp[1]-temp[2];
        m2=temp[2]-temp[0];
        m0=m0>0?m0:(-m0);
        m1=m1>0?m1:(-m1);
        m2=m2>0?m2:(-m2);
        if(m0>THRESHOLD&&m1>THRESHOLD&&m2>THRESHOLD) 
            return 0;
        
        if(m0<m1)
        {
            if(m2<m0) 
                screen.y=(temp[0]+temp[2])/2;
            else 
                screen.y=(temp[0]+temp[1])/2;	
        }
        else if(m2<m1) 
            screen.y=(temp[0]+temp[2])/2;
        else
            screen.y=(temp[1]+temp[2])/2;
        
        return &screen;
    }
    
    else if(count>1)
    {
        screen.x=buffer[0][0];
        screen.y=buffer[1][0];
        return &screen;
    }  
    return 0; 
}




/******************************************************
* 函数名：Read_2046
* 描述  ：得到简单滤波之后的X Y
* 输入  : 无
* 输出  ：Coordinate结构体地址
* 举例  ：无
* 注意  ：”画板应用实例"专用,不是很精准，但是速度比较快
*********************************************************/    
Coordinate *Read_2046_2(void)
{
    static Coordinate  screen2;
    int TP_X[1],TP_Y[1];
    uint8_t count=0;
    int buffer[2][10]={{0},{0}};  /*坐标X和Y进行多次采样*/
    int min_x,max_x;
    int min_y,max_y;
    int	i=0;
    
    do					       				
    {		/* 循环采样10次 */   
        Touch_GetAdXY(TP_X,TP_Y);  
        buffer[0][count]=TP_X[0];  
        buffer[1][count]=TP_Y[0];
        count++;  
    }	/*用户点击触摸屏时即TP_INT_IN信号为低 并且 count<10*/
    while(!INT_IN_2046&& count<10);
    
		/*如果触笔弹起*/
    if(INT_IN_2046)						
    {
				/*中断标志复位*/
        touch_flag = 0;					 
    }
		
		/*如果成功采样10个样本*/
    if(count ==10)		 					
    {
        max_x=min_x=buffer[0][0];
        max_y=min_y=buffer[1][0];
        for(i=1; i<10; i++)
        {
            if(buffer[0][i]<min_x)
            {
                min_x=buffer[0][i];
            }
            else
            if(buffer[0][i]>max_x)
            {
                max_x = buffer[0][i];
            }
        }
        
        for(i=1; i<10; i++)
        {
            if(buffer[1][i]<min_y)
            {
                min_y=buffer[1][i];
            }
            else
            if(buffer[1][i]>max_y)
            {
                max_y = buffer[1][i];
            }
        }

				/*去除最小值和最大值之后求平均值*/
        screen2.x=(buffer[0][0]+buffer[0][1]+buffer[0][2]+buffer[0][3]+buffer[0][4]+buffer[0][5]+buffer[0][6]+buffer[0][7]+buffer[0][8]+buffer[0][9]-min_x-max_x)>>3;
        screen2.y=(buffer[1][0]+buffer[1][1]+buffer[1][2]+buffer[1][3]+buffer[1][4]+buffer[1][5]+buffer[1][6]+buffer[1][7]+buffer[1][8]+buffer[1][9]-min_y-max_y)>>3; 
        
        return &screen2;
    }
    
    return 0;
    
} 






/******************************************************
* 函数名：Cal_touch_para
* 描述  ：计算出触摸屏到液晶屏坐标变换的转换函数的 K A B C D E F系数
* 输入  : 无
* 输出  ：返回1表示成功 0失败
* 举例  ：无
* 注意  ：只有在LCD和触摸屏间的误差角度非常小时,才能运用下面公式
*********************************************************/    
FunctionalState Cal_touch_para( Coordinate * displayPtr,
                                Coordinate * screenPtr,
                                Parameter * para)
{
    
    FunctionalState retTHRESHOLD = ENABLE ;

    /* K＝(X0－X2) (Y1－Y2)－(X1－X2) (Y0－Y2) */
    para->Divider = ((screenPtr[0].x - screenPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) - 
                    ((screenPtr[1].x - screenPtr[2].x) * (screenPtr[0].y - screenPtr[2].y)) ;
    
		if( para->Divider == 0 )
    {
        retTHRESHOLD = DISABLE;
    }
    else
    {
        /* A＝((XD0－XD2) (Y1－Y2)－(XD1－XD2) (Y0－Y2))／K	*/
        para->An = ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) - 
                   ((displayPtr[1].x - displayPtr[2].x) * (screenPtr[0].y - screenPtr[2].y));
        
        /* B＝((X0－X2) (XD1－XD2)－(XD0－XD2) (X1－X2))／K	*/
        para->Bn = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].x - displayPtr[2].x)) - 
                   ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].x - screenPtr[2].x));
        
        /* C＝(Y0(X2XD1－X1XD2)+Y1(X0XD2－X2XD0)+Y2(X1XD0－X0XD1))／K */
        para->Cn = (screenPtr[2].x * displayPtr[1].x - screenPtr[1].x * displayPtr[2].x) * screenPtr[0].y +
                   (screenPtr[0].x * displayPtr[2].x - screenPtr[2].x * displayPtr[0].x) * screenPtr[1].y +
                   (screenPtr[1].x * displayPtr[0].x - screenPtr[0].x * displayPtr[1].x) * screenPtr[2].y ;
        
        /* D＝((YD0－YD2) (Y1－Y2)－(YD1－YD2) (Y0－Y2))／K	*/
        para->Dn = ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].y - screenPtr[2].y)) - 
                   ((displayPtr[1].y - displayPtr[2].y) * (screenPtr[0].y - screenPtr[2].y)) ;
        
        /* E＝((X0－X2) (YD1－YD2)－(YD0－YD2) (X1－X2))／K	*/
        para->En = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].y - displayPtr[2].y)) - 
                   ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].x - screenPtr[2].x)) ;
        
        
        /* F＝(Y0(X2YD1－X1YD2)+Y1(X0YD2－X2YD0)+Y2(X1YD0－X0YD1))／K */
        para->Fn = (screenPtr[2].x * displayPtr[1].y - screenPtr[1].x * displayPtr[2].y) * screenPtr[0].y +
                   (screenPtr[0].x * displayPtr[2].y - screenPtr[2].x * displayPtr[0].y) * screenPtr[1].y +
                   (screenPtr[1].x * displayPtr[0].y - screenPtr[0].x * displayPtr[1].y) * screenPtr[2].y;
        
    }
    return( retTHRESHOLD ) ;
}






/******************************************************
* 函数名：Get_touch_point
* 描述  ：通过 K A B C D E F 把通道X Y的值转换为液晶屏坐标
* 输入  : 无
* 输出  ：返回1表示成功 0失败
* 举例  ：无
* 注意  ：如果获取的触点信息有误，将返回DISABLE
*********************************************************/    
long double linear=0 ;
long double aa1=0,bb1=0,cc1=0,aa2=0,bb2=0,cc2=0;
FunctionalState Get_touch_point(Coordinate * displayPtr,
                                Coordinate * screenPtr,
                                Parameter * para )
{
    FunctionalState retTHRESHOLD =ENABLE ;
    
    if(screenPtr==0)
    {
        /*如果获取的触点信息有误，则返回DISABLE*/
				retTHRESHOLD = DISABLE;			
    }
    else
        if( para->Divider != 0 )
        {		 
        
        
        displayPtr->x = ( (aa1 * screenPtr->x) + 
                          (bb1 * screenPtr->y) + 
                          cc1 
                          ) ;
        
        displayPtr->y = ((aa2 * screenPtr->x) + 
                         (bb2 * screenPtr->y) + 
                         cc2 
                         );
    }
    else
    {
        retTHRESHOLD = DISABLE;
    }
    return(retTHRESHOLD);
} 




/******************************************************
* 函数名：Init_Palette0
* 描述  ：界面初始化
* 输入  : 无
* 输出  ：无
* 举例  ：无
* 注意  ：“触摸画板实例”专用
*********************************************************/    
void Init_Palette0(void)
{	//int i=0;
  	Set_direction(0);	 //设置为横屏 
	LCD_Rectangle(0,0,320,240,WHITE);	/*清屏*/ //  BLACK	   GRED

	LCD_Rectangle(50, 50,220,140,BLACK);   		//
	LCD_Rectangle(52, 52,216,136,WHITE);		//||

	LCD_Str_6x12_O(80, 80,"The system is starting!!!", BLUE);

	LCD_Rectangle(90, 120,130,20,BLACK);   		//
	LCD_Rectangle(91, 121,128,18,WHITE);		//||
	delay_ms(1000);
	LCD_Rectangle(90, 120,26,20,BLUE);			//||
		delay_ms(1000);
	LCD_Rectangle(90, 120,2*26,20,BLUE);		//||	  显示进度条，需要修改为根据GPS信号
   	delay_ms(1000);
	LCD_Rectangle(90, 120,3*26,20,BLUE);		//||	   有无来切换
		delay_ms(1000);
	LCD_Rectangle(90, 120,4*26,20,BLUE);	   	//||
		delay_ms(1000);
	LCD_Rectangle(90, 120,5*26,20,BLUE);	   	//
	delay_ms(1000);
  
} 

/******************************************************
* 函数名：Init_Palette_CH 中文
* 描述  ： 主界面显示函数（背景色，当前语言）
* 输入  : 无
* 输出  ：无
* 举例  ：无
* 注意  ：“触摸画板实例”专用
*********************************************************/   
void Init_Palette(void)
{	
  	
	LCD_Rectangle(0,0,320,240,BLACK);	/*清屏*/ //  BLACK	   GRED

	//LCD_Str_6x12_O(50, 20,"LCD control system", 0);
		
	LCD_Rectangle(15,15,40,40,BLACK);
	LCD_Rectangle(17,17,36,36,shuzi_color);
	LCD_Num_12x24_O(28,24,1,BLACK);

	LCD_Rectangle(15,70,40,40,BLACK);
	LCD_Rectangle(17,72,36,36,shuzi_color);
	LCD_Num_12x24_O(28,79,4,BLACK);

	LCD_Rectangle(15,125,40,40,BLACK);
	LCD_Rectangle(17,127,36,36,shuzi_color);
	LCD_Num_12x24_O(28,134,7,BLACK);

	LCD_Rectangle(15,180,40,40,BLACK);
	LCD_Rectangle(17,182,36,36,shuzi_color);
	LCD_Num_12x24_O(28,189,0,BLACK);

	LCD_Rectangle(70,15,40,40,BLACK);
	LCD_Rectangle(72,17,36,36,shuzi_color);
	LCD_Num_12x24_O(83,24,2,BLACK);

	LCD_Rectangle(70,70,40,40,BLACK);
	LCD_Rectangle(72,72,36,36,shuzi_color);
	LCD_Num_12x24_O(83,79,5,BLACK);

	LCD_Rectangle(70,125,40,40,BLACK);
	LCD_Rectangle(72,127,36,36,shuzi_color);
	LCD_Num_12x24_O(83,134,8,BLACK);

	LCD_Rectangle(70,180,40,40,BLACK);
	LCD_Rectangle(72,182,36,36,shuzi_color);
	LCD_Str_6x12_O(86, 193,"C", 0);	//

	LCD_Rectangle(125,15,40,40,BLACK);
	LCD_Rectangle(127,17,36,36,shuzi_color);
	LCD_Num_12x24_O(138,24,3,BLACK);

	LCD_Rectangle(125,70,40,40,BLACK);
	LCD_Rectangle(127,72,36,36,shuzi_color);
	LCD_Num_12x24_O(138,79,6,BLACK);

	LCD_Rectangle(125,125,40,40,BLACK);
	LCD_Rectangle(127,127,36,36,shuzi_color);
	LCD_Num_12x24_O(138,134,9,BLACK);

	LCD_Rectangle(125,180,40,40,BLACK);
	LCD_Rectangle(127,182,36,36,WHITE);


	LCD_Rectangle(180,15,120,50,BLACK);  //显示区域 1
	LCD_Rectangle(183,18,114,44,shuzi_color);
	LCD_Rectangle(180,75,120,50,BLACK);  //显示区域 2
	LCD_Rectangle(183,78,114,44,shuzi_color);

	LCD_Rectangle(200,140,70,35,BLACK);  //功能键
	LCD_Rectangle(202,142,66,31,shuzi_color);
	LCD_Str_6x12_O(220,150,"out", BLACK);

	LCD_Rectangle(200,190,70,35,BLACK);
	LCD_Rectangle(202,192,66,31,shuzi_color);
	LCD_Str_6x12_O(220,200,"back", BLACK);

    delay_ms(50);
    
}


int Palette_draw_point(uint16_t Xpos,uint16_t Ypos)
{
    //static u16 Pen_color=0;	
	int x=20;
	u16 Tmp_xpos = Xpos;

	  if((Ypos>15)&&(Ypos<55))
          {  
		     if((Tmp_xpos>20)&&(Tmp_xpos<60))
			{
			x=0;
			}
			if((Tmp_xpos>75)&&(Tmp_xpos<115))
			{
			x=7;
			}
			 if((Tmp_xpos>130)&&(Tmp_xpos<170))
			{
			x=4;
			}
			if((Tmp_xpos>185)&&(Tmp_xpos<225))
			{
			x=1;
			}
			
		  }

		
        if((Ypos>70)&&(Ypos<110))
          {  
		if((Tmp_xpos>20)&&(Tmp_xpos<60))
			{
			x=10;
			}
			if((Tmp_xpos>75)&&(Tmp_xpos<115))
			{
			x=8;
			}
			 if((Tmp_xpos>130)&&(Tmp_xpos<170))
			{
			x=5;
			}
			if((Tmp_xpos>185)&&(Tmp_xpos<225))
			{
			x=2;
			}
			
		  }

	  if((Ypos>125)&&(Ypos<165))
          {  
		if((Tmp_xpos>20)&&(Tmp_xpos<60))
			{
				;
			}
			if((Tmp_xpos>75)&&(Tmp_xpos<115))
			{
			x=9;
			}
			 if((Tmp_xpos>130)&&(Tmp_xpos<170))
			{
			x=6;
			}
			if((Tmp_xpos>185)&&(Tmp_xpos<225))
			{
			x=3;
			}
			
		  }

	  if((Ypos>200)&&(Ypos<270))
          {  
		if((Tmp_xpos>15)&&(Tmp_xpos<50))
			{
			 x=12;
			}
			if((Tmp_xpos>65)&&(Tmp_xpos<100))
			{
			x=11;
			}
			
		  }

  	   return x;
}	




/******************************************************
* 函数名：Get_touch_place
* 描述  ：获取触摸LCD位置
* 输入  : 无
* 输出  ：无
* 举例  ：无
* 注意  ：如果触摸笔没有按下,将直接返回1
*********************************************************/    
char  Get_touch_place(u16 *x, u16 *y)
{		
		/*如果触笔有按下*/
    if(touch_flag == 1)				
    {
        if(Get_touch_point(&display, Read_2046(), &touch_para ) !=DISABLE)
        {
            
            *x = display.x ;
            *y = display.y ;
            return 0;
        }
    }
    
    return 1;				
    
}






/******************************************************
* 函数名：Touchl_Calibrate
* 描述  ：触摸屏校正函数
* 输入  : 无
* 输出  ：0	---	校正成功
					1	---	校正失败
* 举例  ：无
* 注意  ：无
*********************************************************/    
int Touchl_Calibrate(void)
{
    uint8_t i;
    u16 test_x=0, test_y=0;
    u16 gap_x=0, gap_y=0;
    Coordinate * Ptr;
	
   // delay_init();
    Set_direction(0);
    for(i=0;i<4;i++)
    {
        LCD_Rectangle(0,0,320,240,CAL_BACKGROUND_COLOR);  //使整个屏幕显示背景颜色			 
        LCD_Str_6x12_O(10, 10,"Touch Calibrate", 0);  //显示提示信息
		LCD_Num_6x12_O(10,25, i+1,  0);				  //显示触点次数
        
        delay_ms(500);
        DrawCross(DisplaySample[i].x,DisplaySample[i].y);  //显示校正用的“十”字
		
       do
        {
            Ptr=Read_2046();  //读取TSC2046数据到变量ptr
        }
        while( Ptr == (void*)0 );  //当ptr为空时表示没有触点被按下
        ScreenSample[i].x= Ptr->x; 	  //把读取的原始数据存放到ScreenSample结构体
        ScreenSample[i].y= Ptr->y;

    }

		/* 用原始参数计算出 原始参数与坐标的转换系数。 */
    Cal_touch_para( &DisplaySample[0],&ScreenSample[0],&touch_para ) ;  	   
    
		/*计算X值*/
    test_x = ( (touch_para.An * ScreenSample[3].x) + 
               (touch_para.Bn * ScreenSample[3].y) + 
               touch_para.Cn 
               ) / touch_para.Divider ;			 
    
		/*计算Y值*/
    test_y = ( (touch_para.Dn * ScreenSample[3].x) + 
               (touch_para.En * ScreenSample[3].y) + 
               touch_para.Fn 
               ) / touch_para.Divider ;			 
    
    gap_x = (test_x > DisplaySample[3].x)?(test_x - DisplaySample[3].x):(DisplaySample[3].x - test_x);
    gap_x = (test_y > DisplaySample[3].y)?(test_y - DisplaySample[3].y):(DisplaySample[3].y - test_y);
    

     LCD_Rectangle(0,0,320,240,CAL_BACKGROUND_COLOR);
    if((gap_x>11)||(gap_y>11))
    {

        LCD_Str_6x12_O(100, 100,"Calibrate fail", 0);
		LCD_Str_6x12_O(100, 120,"  try again   ", 0);
		delay_ms(2000);
        return 1;
    }
    
    
    aa1 = (touch_para.An*1.0)/touch_para.Divider;
    bb1 = (touch_para.Bn*1.0)/touch_para.Divider;
    cc1 = (touch_para.Cn*1.0)/touch_para.Divider;
    
    aa2 = (touch_para.Dn*1.0)/touch_para.Divider;
    bb2 = (touch_para.En*1.0)/touch_para.Divider;
    cc2 = (touch_para.Fn*1.0)/touch_para.Divider;
    
	LCD_Str_6x12_O(100, 100,"Calibrate Success", 0);
    delay_ms(1000);
    
    return 0;    
}

/******************* CHD1807 Team *****END OF FILE************/

