/***************************飞音云电子****************************
**  工程名称：YS-V0.7语音识别模块驱动程序
**	CPU: STC11L08XE
**	晶振：22.1184MHZ
**	波特率：9600 bit/S
**	配套产品信息：YS-V0.7语音识别开发板
**                http://yuesheng001.taobao.com
**  作者：zdings
**  联系：751956552@qq.com
**  修改日期：2013.9.13
**  说明：口令模式： 即每次识别时都需要说“小杰”这个口令 ，才能够进行下一级的识别
/***************************飞音云电子******************************/
#include "config.h"
#define FOSC 22118400L      //System frequency
uint32_t baud=9600;           //UART baudrate
/************************************************************************
函 数 名： 串口初始化
功能描述： STC10L08XE 单片机串口初始化函数
返回函数： none
其他说明： none
**************************************************************************/
void UartIni(void)
{
    SCON = 0x50;            //8-bit variable UART
    TMOD = 0x20;            //Set Timer1 as 8-bit auto reload mode
    TH1 = TL1 = -(FOSC/12/32/baud); //Set auto-reload vaule
    TR1 = 1;                //Timer1 start run
    ES = 1;                 //Enable UART interrupt
    EA = 1;                 //Open master interrupt switch
}
/************************************************************************
功能描述： 	串口发送一字节数据
入口参数：	DAT:带发送的数据
返 回 值： 	none
其他说明：	none
**************************************************************************/
void UARTSendByte(uint8_t DAT)
{
	ES  =  0;
	TI=0;
	SBUF = DAT;
	while(TI==0);
	TI=0;
	ES = 1;
}
/************************************************************************
功能描述： 串口发送字符串数据
入口参数： 	*DAT：字符串指针
返 回 值： none
其他说明： API 供外部使用，直观！
**************************************************************************/
void PrintCom(uint8_t *DAT)
{
	while(*DAT)
	{
	 	UARTSendByte(*DAT++);
	}	
}

void PrintCom_len(uint8_t *DAT,uint8_t len)
{
	uint8_t i;
	for(i=0;i<len;i++)
	{
	 	UARTSendByte(*DAT++);
	}	
}


/**************芯片设置命令*********************/
uint8_t SYN_StopCom[]={0xFD,0X00,0X02,0X02,0XFD};//停止合成
uint8_t SYN_SuspendCom[]={0XFD,0X00,0X02,0X03,0XFC};//暂停合成
uint8_t SYN_RecoverCom[]={0XFD,0X00,0X02,0X04,0XFB};//恢复合成
uint8_t SYN_ChackCom[]={0XFD,0X00,0X02,0X21,0XDE};//状态查询
uint8_t SYN_PowerDownCom[]={0XFD,0X00,0X02,0X88,0X77};//进入POWER DOWN 状态命令

/***********************************************/



/***********************************************************
* 名    称：  YS-SYN6288 文本合成函数
* 功    能：  发送合成文本到SYN6288芯片进行合成播放
* 入口参数：Music(背景音乐选择):0无背景音乐。1-15：相关背景音乐
            *HZdata:文本指针变量 
* 出口参数：
* 说    明： 本函数只用于文本合成，具备背景音乐选择。默认波特率9600bps。					 
* 调用方法：例： SYN_FrameInfo（0，“乐声电子科技”）；
**********************************************************/
void SYN_FrameInfo(uint8_t Music,uint8_t *HZdata)
{
/****************需要发送的文本**********************************/ 
		 unsigned  char  Frame_Info[5];
         unsigned  char  HZ_Length;  
		 unsigned  char  ecc  = 0;  			//定义校验字节
	     unsigned  int i=0; 
		 HZ_Length =strlen(HZdata); 			//需要发送文本的长度
 
/*****************帧固定配置信息**************************************/           
		 Frame_Info[0] = 0xFD ; 			//构造帧头FD
		 Frame_Info[1] = 0x00 ; 			//构造数据区长度的高字节
		 Frame_Info[2] = HZ_Length + 3; 		//构造数据区长度的低字节
		 Frame_Info[3] = 0x01 ; 			//构造命令字：合成播放命令		 		 
		 Frame_Info[4] = 0x01 | Music<<4 ;  //构造命令参数：背景音乐设定

/*******************校验码计算***************************************/		 
		 for(i = 0; i<5; i++)   				//依次发送构造好的5个帧头字节
	     {  
	         ecc=ecc^(Frame_Info[i]);		//对发送的字节进行异或校验	
	     }

	   	 for(i= 0; i<HZ_Length; i++)   		//依次发送待合成的文本数据
	     {  
	         ecc=ecc^(HZdata[i]); 				//对发送的字节进行异或校验		
	     }		 
/*******************发送帧信息***************************************/		  
		 PrintCom_len(Frame_Info,5); //发送帧配置
		 PrintCom_len(HZdata,HZ_Length); //发送文本
		 UARTSendByte(ecc);    //发送纠错码
}


/***********************************************************
* 名    称： void  main(void)
* 功    能： 主函数	程序入口
* 入口参数： *Info_data:固定的配置信息变量 
* 出口参数：
* 说    明：本函数用于配置，停止合成、暂停合成等设置 ，默认波特率9600bps。					 
* 调用方法：通过调用已经定义的相关数组进行配置。 
**********************************************************/
void YS_SYN_Set(uint8_t *Info_data)
{
	uint8_t Com_Len;
	Com_Len =strlen(Info_data);
	PrintCom_len(Info_data,Com_Len);
}



