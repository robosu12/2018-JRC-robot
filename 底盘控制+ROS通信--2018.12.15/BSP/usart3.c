#include "main.h"
#include "FIFO.h"
#include "protocal.h"

/*-----USART3_TX-----PB10-----*/
/*-----USART3_RX-----PB11-----*/

//FIFO_S_t* UART_TranFifo;

float Medie_Angle = 0;  //-0.8

struct SAcc 		stcAcc; 
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;
struct SQ 			stcQ;

extern uint32_t system_time;   //系统时间 单位ms


int fputc(int ch, FILE *f)
{
    while (USART_GetFlagStatus(UART4,USART_FLAG_TC) == RESET);
    USART_SendData(UART4, (uint8_t)ch);
    return ch;
}
void USARTx_PutChar(USART_TypeDef* USARTx,u8 ch)
{
   USART_SendData(USARTx, (u8)ch);
   while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);   //??????
}

/***********************************************************************************/
union xxx
{
char c[4];
float v;
}yV;
union xxxx
{
char d[4];
float a;
}yA;


extern float Robot_Yaw_Angel,Robot_Roll_Angel,Robot_Pitch_Angel;
extern float Ax, Ay, Az, Gx, Gy, Gz, Q0,Q1,Q2,Q3;
extern float Gyro_Balance,Gyro_Turn;
extern int target_speed;
extern u8 imu_data_flag;
extern union IMU  //
{
    float float_type;
    unsigned char char_type[4];
}ACC_x,ACC_y,ACC_z,GY_x,GY_y,GY_z,QUAT_0,QUAT_1,QUAT_2,QUAT_3,Roll_Angle,Pitch_Angle,Yaw_Angle;;

////////////////////////////////////////////////////////////////////////////////////////

void IMU_data_receive_prepare(u8 data)
{
	static u8 RxBuffer[10];
	static u8 _data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0X55)
	{
		state=1;
	}
	else if(state==1&&data==0X51)
	{
		state=2;
		_data_cnt=0;
	}
	else if(state==1&&data==0X52)
	{
		state=3;
		_data_cnt=0;
	}
	else if(state==1&&data==0X53)
	{
		state=4;
		_data_cnt=0;
	}
	else if(state==1&&data==0X59)
	{
		state=5;
		_data_cnt=0;
	}
	else if(state>1)
	{
		RxBuffer[_data_cnt++]=data;
		if(_data_cnt>=8)
		{		
			//data proccess
			imu_data_flag=1;
			if(state==2)
			{
				memcpy(&stcAcc,&RxBuffer[0],6);
				ACC_x.float_type = 1.0*stcAcc.a[0]/32768*16*9.8;  // m / s2
				ACC_y.float_type = 1.0*stcAcc.a[1]/32768*16*9.8;
				ACC_z.float_type = 1.0*stcAcc.a[2]/32768*16*9.8;
			}
			else if(state==3)
			{
				memcpy(&stcGyro,&RxBuffer[0],6);				
				GY_x.float_type = 1.0*stcGyro.w[0]/32768*2000/180*PI; // rad / s 
				GY_y.float_type = 1.0*stcGyro.w[1]/32768*2000/180*PI;
				GY_z.float_type = 1.0*stcGyro.w[2]/32768*2000/180*PI;			
			}
			else if(state==4)
			{
				memcpy(&stcAngle,&RxBuffer[0],6);
				Roll_Angle.float_type = 1.0*stcAngle.Angle[0]/32768*180 - Medie_Angle;	
				Pitch_Angle.float_type = 1.0*stcAngle.Angle[1]/32768*180 - Medie_Angle;	
				Yaw_Angle.float_type = 1.0*stcAngle.Angle[2]/32768*180 - Medie_Angle;	
				Robot_Yaw_Angel = Yaw_Angle.float_type;
			} 
			else if(state==5)
			{
				memcpy(&stcQ,&RxBuffer[0],8);	
				QUAT_0.float_type = 1.0*stcQ.q[0]/32768;
				QUAT_1.float_type = 1.0*stcQ.q[1]/32768;
				QUAT_2.float_type = 1.0*stcQ.q[2]/32768;
				QUAT_3.float_type = 1.0*stcQ.q[3]/32768;
			}
				
			state = 0;
		}
    
	}
	else
	{
		state = 0;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////

extern int wire_data[20],wire_data_flag;

void remote_data_receive_prepare(int data)
{
	static u8 RxBuffer[40];
	static u8 _data_len = 0,data_00_cnt=0;
	static u8 state = 0;
	
	if(state==0&&data==0x00)
	{
		data_00_cnt++;
		if(data_00_cnt==12)
		{
			data_00_cnt=0;
			_data_len=0;
			state=1;
		}	
	}
	else if(state==1&&_data_len<6)
	{
		RxBuffer[_data_len]=data;
		if(_data_len==5)
		{
			  wire_data[0] = (RxBuffer[0]| (RxBuffer[1] << 8)) & 0x07ff;           //!< Channel 0			  
        wire_data[1] = ((RxBuffer[1] >> 3) | (RxBuffer[2] << 5)) & 0x07ff;   //!< Channel 1 	  			
        wire_data[2] = ((RxBuffer[2] >> 6) | (RxBuffer[3] << 2)|(RxBuffer[4] << 10))&0x07ff;//Channel 2                       
        wire_data[3] = ((RxBuffer[4] >> 1) | (RxBuffer[5] << 7)) & 0x07ff;   //!< Channel 3 		  			
        wire_data[4] = ((RxBuffer[5] >> 4)& 0x000C) >> 2;                          //!< Switch left 
        wire_data[5] = ((RxBuffer[5] >> 4)& 0x0003);                               //!< Switch right 	
			   
			  //wire_data[0] = 1.0*(wire_data[0] - 1024)/5.2;
			  wire_data[0] = wire_data[0] - 1024;
			  wire_data[1] = wire_data[1] - 1024;
			  wire_data[2] = wire_data[2] - 1024;
			  wire_data[3] = wire_data[3] - 1024;
			  
			  wire_data_flag=1;
			
			state = 0;
		}
    _data_len++;
	}
	else
	{
		state = 0;
		data_00_cnt=0;
		_data_len=0;
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////// siyi_remote_data_receive_preccess /////////////////////////////////////////////////// 
uint16_t remote_channel_data[20];
u8 SIYI_Remote_data_buf[30];
u8 SIYI_Remote_Data_Flag;
int RC[20];

void siyi_remote_data_process(uint8_t *raw_data)
{
	uint8_t bitsToRead=3; // bitsToRead
  uint8_t bitsToShift;
  uint8_t startByte=22;
  uint8_t channelId=15;
	uint8_t i=0;
	

  while(startByte>1) 
  {
    remote_channel_data[channelId] = raw_data[startByte];

    if(bitsToRead<8)
    {
      remote_channel_data[channelId] = remote_channel_data[channelId]<<bitsToRead;
      bitsToShift=8-bitsToRead;
      remote_channel_data[channelId]+=(raw_data[startByte-1]>>bitsToShift);
    }
    else if(bitsToRead==8)
    {
      remote_channel_data[channelId] = remote_channel_data[channelId]<<bitsToRead;
      remote_channel_data[channelId]+=raw_data[startByte-1];
      startByte--;
			bitsToRead-=8;
    }
		else
		{
			remote_channel_data[channelId] = remote_channel_data[channelId]<<bitsToRead;
			bitsToShift=bitsToRead-8;
      remote_channel_data[channelId]+=(raw_data[startByte-1]<<bitsToShift);
      startByte--;
      bitsToRead-=8;
      bitsToShift=8-bitsToRead;
      remote_channel_data[channelId]+=(raw_data[startByte-1]>>bitsToShift);
		}

    remote_channel_data[channelId]&=0x7FF;

    channelId--;
    startByte--;
    bitsToRead+=3;
  }
	
	for(i=0;i<16;i++)
	{
		RC[i] = remote_channel_data[i] - RC_middle; //max:+800,min:-800 
	}
	
}


void siyi_remote_data_receive_prepare(uint8_t data)
{
	static u8 time_diff,data_count;
	static uint32_t last_time;
	
	time_diff = system_time - last_time;
	
	if(time_diff>2)
	{	
		if(SIYI_Remote_data_buf[0]==0x0f&&data_count==25&&SIYI_Remote_data_buf[24]==0x00)
		{
			//siyi_remote_data_process(SIYI_Remote_data_buf);
			SIYI_Remote_Data_Flag=1;
		}
		data_count=0;
	}
	
	SIYI_Remote_data_buf[data_count++] = data;
	last_time = system_time;
}

////------------------ pad data process -------------------------////

u8 Pad_App_Data_Buf[40];
u8 Pad_App_Data_Flag;
void Pad_App_Data_process(u8 data)  // target IP : 192.168.2.254
{
	static u8 data_count;
	static u8 state;
	
	if(state==0&&data==0x55)
	{
		state=1;
	}
	else if(state==1&&data==0xAA)
	{
		state=2;
		data_count=0;
	}
	else if(state==2)
	{
		Pad_App_Data_Buf[data_count++] = data;
		
		if(data_count>=10)
		{
			Pad_App_Data_Flag=1;
			state=0;
		}
	}
	else
	{
		state=0;
	}
}

////------------------ pad data process -------------------------////

u8 Control_Platform_Data_Buf[40];
u8 Control_Platform_Data_Flag;
void Control_Platform_Data_process(u8 data)
{
	static u8 data_count;
	static u8 state;
	
	if(state==0&&data==0x53)
	{
		state=1;
	}
	else if(state==1&&data==0x49)
	{
		state=2;
		data_count=0;
	}
	else if(state==2)
	{
		Control_Platform_Data_Buf[data_count++] = data;
		
		if(data_count>=14)
		{
			Control_Platform_Data_Flag = 1;
			state=0;
		}
	}
	else
	{
		state=0;
	}
}

////------------------ ROS data process -------------------------////

extern u8 ROS_Control_Data_Buf[30];
extern u8 ROS_Control_Data_Flag;
extern u8 ROS_Target_Velocity_Flag,ROS_Target_Position_Flag;

void ROS_Control_Data_Prepare(u8 data)
{
	static u8 data_count;
	static u8 state;
	
	if(state==0&&data==0x53)
	{
		state=1;
	}
	else if(state==1&&data==0x49)
	{
		state=2;
		data_count=0;
		ROS_Target_Velocity_Flag=1;
		ROS_Target_Position_Flag=0;
	}
	else if(state==1&&data==0x4a)
	{
		state=3;
		data_count=0;
		ROS_Target_Velocity_Flag=0;
		ROS_Target_Position_Flag=1;
	}
	else if(state==2)
	{
		ROS_Control_Data_Buf[data_count++] = data;
		
		if(data_count>=20)
		{
			ROS_Control_Data_Flag = 1;
			state=0;
		}
	}
	else
	{
		state=0;
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////

void USART3_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// USART1 //////////////////////////////////////////////////////////////////////////////
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	//配置PA9作为USART1　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//配置PA10作为USART1　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	//配置USART1
	//中断被屏蔽了
	USART_InitStructure.USART_BaudRate = 115200;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	USART_Init(USART1, &USART_InitStructure);
	
	//使能USART1接收中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	//使能USART1
	USART_Cmd(USART1, ENABLE); 	
	
	// USART2 //////////////////////////////////////////////////////////////////////////////

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
	
	//配置PA9作为USART1　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	//配置PA10作为USART1　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	//配置USART2
	//中断被屏蔽了
	USART_InitStructure.USART_BaudRate = 115200;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	USART_Init(USART2, &USART_InitStructure);
	
	//使能USART2接收中断
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART2, ENABLE); 

    // USART3 //////////////////////////////////////////////////////////////////////////////

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
	
	//配置PA9作为USART1　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	//配置PA10作为USART1　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	//配置USART3
	//中断被屏蔽了
	USART_InitStructure.USART_BaudRate = 115200;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_2;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	USART_Init(USART3, &USART_InitStructure);
	
	//使能USART2接收中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART3, ENABLE); 
	
	// UART4 //////////////////////////////////////////////////////////////////////////////

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); //开启UART4时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
	
	//配置PA9作为USART1　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//配置PA10作为USART1　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
	//配置USART4
	//中断被屏蔽了
	USART_InitStructure.USART_BaudRate = 115200;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	USART_Init(UART4, &USART_InitStructure);
	
	//使能USART3接收中断
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	//使能USART3
	USART_Cmd(UART4, ENABLE); 
	
	// UART5 //////////////////////////////////////////////////////////////////////////////

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //开启UART5时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2,  GPIO_AF_UART5);
	
	//配置PA9作为USART1　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//配置PA10作为USART1　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	//配置USART5
	//中断被屏蔽了
	USART_InitStructure.USART_BaudRate = 100000;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	USART_Init(UART5, &USART_InitStructure);
	
	//使能USART3接收中断
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	//使能USART3
	USART_Cmd(UART5, ENABLE); 
	
	// USART6 //////////////////////////////////////////////////////////////////////////////

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); //开启USART6时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

	//配置PA9作为USART1　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//配置PA10作为USART1　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
	//配置USART1
	//中断被屏蔽了
	USART_InitStructure.USART_BaudRate = 115200;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	USART_Init(USART6, &USART_InitStructure);
	
	//使能USART1接收中断
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
	//使能USART1
	USART_Cmd(USART6, ENABLE); 	
}

void USART1_IRQHandler(void)
{  
	  uint8_t com_data;
	 //接收中断
	if( USART_GetITStatus(USART1,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//清除中断标志

		com_data = USART_ReceiveData(USART1);
		
		//remote_data_receive_prepare(com_data);
		
		Pad_App_Data_process(com_data);
		
//		USART_SendData(UART4, com_data);//max:AC min:46
//		while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
		
		//Control_Platform_Data_process(com_data);
		
//		USART_SendData(USART1, com_data);
//		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);   //		
	} 					
}

void USART2_IRQHandler(void)
{  
	  uint8_t com_data = 0;
	 //接收中断
	if( USART_GetITStatus(USART2,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);//清除中断标志

		com_data = USART2->DR;
		
		IMU_data_receive_prepare(com_data);
		
//		USART_SendData(UART5, 0xAA);
//		while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);   //
//		
//		USART_SendData(UART5, pitch_angle+128);
//		while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);   //
//		
//		USART_SendData(UART5, Gyro_Balance+128);
//		while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);   //
//			
//		USART_SendData(UART5, Gyro_Turn+128);
//		while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);   //
	} 					
}


void USART3_IRQHandler(void)
{  
	  u8 temp_data;		
	
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)    //接收中断
    {	
			USART_ClearITPendingBit(USART3,USART_IT_RXNE);//清除中断标志
			
//			temp_data = USART_ReceiveData(USART3);
//			
//		  siyi_remote_data_receive_prepare(temp_data);
			
			//IMU_data_receive_prepare(temp_data);
        
    }		
}

void UART4_IRQHandler(void)
{  
	uint8_t temp_data;
	 //接收中断
	if( USART_GetITStatus(UART4,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART4,USART_IT_RXNE);//清除中断标志

		temp_data = USART_ReceiveData(UART4);
		
		ROS_Control_Data_Prepare(temp_data);
		//Control_Platform_Data_process(temp_data);
		
//		USART_SendData(UART4, temp_data);
//		while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET); 
	} 					
}

void UART5_IRQHandler(void)
{ 
  uint8_t temp_data;	
	 //接收中断
	if( USART_GetITStatus(UART5,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART5,USART_IT_RXNE);//清除中断标志	
		
		temp_data = USART_ReceiveData(UART5);
		
		//siyi_remote_data_receive_prepare(temp_data);
		
//		USART_SendData(UART4, temp_data);
//		while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);   //
		//temp_data = USART_ReceiveData(UART5);	
		//remote_data_receive_prepare(temp_data);
	} 						
}

void USART6_IRQHandler(void)
{  
	 //接收中断
	if( USART_GetITStatus(USART6,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART6,USART_IT_RXNE);//清除中断标志
		
	} 				
		
}

////////////////////////////////

