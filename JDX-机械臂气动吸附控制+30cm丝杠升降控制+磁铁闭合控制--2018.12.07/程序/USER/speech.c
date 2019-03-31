/***********************************************************
**
** 追梦实验室 SYN6288 中文语音合成开发板测试程序演示版
**
** 本程序用于 AT89S51 单片机，若用于其他单片机请自行修改
**
** 作    者：CIKY & SPP From Dreamon-II Labs.
**
** 版权所有：追梦计算机及电子技术实验室
**
** 最后修改：2012-8-21
**
***********************************************************/
#include "string.h"
#include "stm32f10x_usart.h"
#include "speech.h"



#define HEADLEN       5  //数据包头的长度
#define BKM_OFFSET    4  //背景音乐命令偏移
#define LEN_OFFSET    2  //长度字节的偏移量（一般不会超过255字节，因此只使用1字节长度）
#define BKM_MAX      15	 //背景音乐数量

//*****************************************************
//sbit BUSY = P1^7;        //开发板上SYN6288的BUSY引脚固定连接到CPU的P3.7端口
uchar nBkm = 0x00;

uchar send_speech[255]=" ";
//*****************************************************



//*****************************************************

//数据包头（0xFD + 2字节长度 + 1字节命令字 + 1字节命令参数)

uchar head[] = {0xfd,0x00,0x00,0x01,0x00};
void Speech(char *buf)
{
	u8 i=0;
	uchar len = 0x00;
	while(buf[++len]);
		for(i = 0; i < HEADLEN; i++)
	{
		if(i == BKM_OFFSET)
			send_speech[i]= nBkm << 3; //写入背景音乐
		else if(i == LEN_OFFSET)
			send_speech[i] = len + 3;
		else
			send_speech[i] = head[i];
   	}
	strcpy(&send_speech[5],buf);
	USART_ITConfig(USART3,USART_IT_TXE,ENABLE);
	

	//发送数据包头(0xFD + 2字节长度 + 1字节命令字 + 1字节命令参数)
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
		    USART_SendData(USART3,Frame_Info[i]);
		    while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET){}//等待发送结束	
	     }

	   	 for(i= 0; i<HZ_Length; i++)   		//依次发送待合成的文本数据
	     {  
	         ecc=ecc^(HZdata[i]); 				//对发送的字节进行异或校验
		    USART_SendData(USART3,HZdata[i]);
		    while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET){}//等待发送结束		
	     }		 
/*******************发送帧信息***************************************/		  
		 //PrintCom_len(Frame_Info,5); //发送帧配置
		 //PrintCom_len(HZdata,HZ_Length); //发送文本
		 //UARTSendByte(ecc);    //发送纠错码
		 USART_SendData(USART3,ecc);
		 while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET){}//等待发送结束
	
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
	//PrintCom_len(Info_data,Com_Len);
}



