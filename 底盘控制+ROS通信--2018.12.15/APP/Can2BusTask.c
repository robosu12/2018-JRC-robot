#include "main.h"
#include "stdio.h"

unsigned int can2_tx_success_flag;

volatile Encoder CM1Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM2Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM3Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM4Encoder = {0,0,0,0,0,0,0,0,0};


u16 left_encoder_send[8],right_encoder_send[8];

////////////////////////////////////////////////////////////////////////////

static void CAN_Delay_Us(unsigned int t)
{
	int i;
	for(i=0;i<t;i++)
	{
		int a=40;
		while(a--);
	}
}

void CAN2_TX_Wait(void)
{
	u8 CAN_Time_Out = 0;
	can2_tx_success_flag = 0;
	while(can2_tx_success_flag == 0)
	{
			CAN_Delay_Us(1);
			CAN_Time_Out++;
			if(CAN_Time_Out>250)
			{
					break;
			}
	}
}

/*
***********************************************************************************************
*Name          :EncoderProcess
*Input         :can message
*Return        :void
*Description   :to get the initiatial encoder of the chassis motor 201 202 203 204
*
*
***********************************************************************************************
*/

void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg)
{
            v->ecd_bias = (msg->Data[0]<<8)|msg->Data[1];  //保存初始编码器值作为偏差  
            v->ecd_value = v->ecd_bias;
            v->last_raw_value = v->ecd_bias;
						v->raw_value = v->ecd_bias;
            v->temp_count++;
}


void EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0]<<8)|msg->Data[1]; // RPM_rate
	v->RPM_rate = (msg->Data[2]<<8)|msg->Data[3]; // RPM_rate
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -5000)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>5000)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//计算得到连续的编码器输出值
	//v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
//	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
//	if(v->buf_count == RATE_BUF_SIZE)
//	{
//		v->buf_count = 0;
//	}
//	//计算速度平均值
//	for(i = 0;i < RATE_BUF_SIZE; i++)
//	{
//		temp_sum += v->rate_buf[i];
//	}
//	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);					
}


/*
************************************************************************************************************************
*Name        : CanReceiveMsgProcess
* Description: This function process the can message representing the encoder data received from the CAN2 bus.
* Arguments  : msg     is a pointer to the can message.
* Returns    : void
* Note(s)    : none
************************************************************************************************************************
*/

void Can2ReceiveMsgProcess(CanRxMsg * msg)
{      
	static int can2_count=0;
	
	if(can2_count<8000)
	{
		can2_count++;
		switch(msg->StdId)
		{
			case CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID:
			{		
				GetEncoderBias(&CM1Encoder ,msg);  //获取到编码器的初始偏差值            						
			}break;
			case CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID:
			{		
				GetEncoderBias(&CM2Encoder ,msg);
			}break;
			case CAN_BUS2_MOTOR3_FEEDBACK_MSG_ID:
			{		
				GetEncoderBias(&CM3Encoder ,msg);   
			}break;
			case CAN_BUS2_MOTOR4_FEEDBACK_MSG_ID:
			{		
				GetEncoderBias(&CM4Encoder ,msg);
			}break;		
						
			default:
			{
			}
		}
	}
	else
	{
		switch(msg->StdId)
		{
			case CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID:
			{		
				EncoderProcess(&CM1Encoder ,msg);       //获取到编码器的初始偏差值            						
			}break;
			case CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID:
			{		
				EncoderProcess(&CM2Encoder ,msg);
			}break;
			case CAN_BUS2_MOTOR3_FEEDBACK_MSG_ID:
			{		
				EncoderProcess(&CM3Encoder ,msg);   
			}break;
			case CAN_BUS2_MOTOR4_FEEDBACK_MSG_ID:
			{		
				EncoderProcess(&CM4Encoder ,msg);
			}break;
						
			default:
			{
			}
		}
	}
	
}

/********************************************************************************
   给底盘电调板发送指令，ID号为0x200８底盘返回ID为0x201-0x204
*********************************************************************************/
void Speed_control(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);
    tx_message.Data[1] = (uint8_t)cm1_iq;
    tx_message.Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm2_iq;
    tx_message.Data[4] = (uint8_t)(cm3_iq >> 8);
    tx_message.Data[5] = (uint8_t)cm3_iq;
    tx_message.Data[6] = (uint8_t)(cm4_iq >> 8);
    tx_message.Data[7] = (uint8_t)cm4_iq;
    CAN_Transmit(CANx,&tx_message);
}


void Set_Balance_Speed(CAN_TypeDef *CANx, int32_t cm1_iq, int32_t cm2_iq)
{
    CanTxMsg tx_message;
	
	  tx_message.StdId = 0x05;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = 0x00;
    tx_message.Data[1] = 0x10;
    tx_message.Data[2] = 0x05;
    tx_message.Data[3] = (uint8_t)(cm1_iq >> 24);
    tx_message.Data[4] = (uint8_t)(cm1_iq >> 16);
    tx_message.Data[5] = (uint8_t)(cm1_iq >> 8);
    tx_message.Data[6] = (uint8_t)cm1_iq;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
	
	  CAN2_TX_Wait();
	
	  tx_message.StdId = 0x06;
	  tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
	  tx_message.Data[0] = 0x00;
    tx_message.Data[1] = 0x10;
    tx_message.Data[2] = 0x05;
    tx_message.Data[3] = (uint8_t)(cm2_iq >> 24);
    tx_message.Data[4] = (uint8_t)(cm2_iq >> 16);
    tx_message.Data[5] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[6] = (uint8_t)cm2_iq;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
		
		CAN2_TX_Wait();
		
//	//////////////////////////////////////////////////////////	

	  Get_Balance_Speed(CAN2);
}

void Get_Balance_Speed(CAN_TypeDef *CANx)
{
    CanTxMsg tx_message;
	
//		
//	//////////////////////////////////////////////////////////	
		tx_message.StdId = 0x05;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = 0x00;
    tx_message.Data[1] = 0x0d;
    tx_message.Data[2] = 0x00;
    tx_message.Data[3] = 0x00;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
	
		CAN2_TX_Wait();
		
		tx_message.StdId = 0x06;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = 0x00;
    tx_message.Data[1] = 0x0d;
    tx_message.Data[2] = 0x00;
    tx_message.Data[3] = 0x00;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
		
		CAN2_TX_Wait();
}

void Set_Track_Speed(CAN_TypeDef *CANx, int32_t cm1_iq, int32_t cm2_iq, int32_t cm3_iq, int32_t cm4_iq)
{
    CanTxMsg tx_message;
	
	  tx_message.StdId = 0x01;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = 0x01;
    tx_message.Data[1] = 0xF4;
    tx_message.Data[2] = 0x05;
    tx_message.Data[3] = (uint8_t)(cm1_iq >> 24);
    tx_message.Data[4] = (uint8_t)(cm1_iq >> 16);
    tx_message.Data[5] = (uint8_t)(cm1_iq >> 8);
    tx_message.Data[6] = (uint8_t)cm1_iq;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
	
		CAN2_TX_Wait();
	
	  tx_message.StdId = 0x02;
	  tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
	  tx_message.Data[0] = 0x01;
    tx_message.Data[1] = 0xF4;
    tx_message.Data[2] = 0x05;
    tx_message.Data[3] = (uint8_t)(cm2_iq >> 24);
    tx_message.Data[4] = (uint8_t)(cm2_iq >> 16);
    tx_message.Data[5] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[6] = (uint8_t)cm2_iq;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);		
		
		CAN2_TX_Wait();
		
		tx_message.StdId = 0x03;
	  tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
	  tx_message.Data[0] = 0x01;
    tx_message.Data[1] = 0xF4;
    tx_message.Data[2] = 0x05;
    tx_message.Data[3] = (uint8_t)(cm3_iq >> 24);
    tx_message.Data[4] = (uint8_t)(cm3_iq >> 16);
    tx_message.Data[5] = (uint8_t)(cm3_iq >> 8);
    tx_message.Data[6] = (uint8_t)cm3_iq;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);	

		CAN2_TX_Wait();
		
		tx_message.StdId = 0x04;
	  tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
	  tx_message.Data[0] = 0x01;
    tx_message.Data[1] = 0xF4;
    tx_message.Data[2] = 0x05;
    tx_message.Data[3] = (uint8_t)(cm4_iq >> 24);
    tx_message.Data[4] = (uint8_t)(cm4_iq >> 16);
    tx_message.Data[5] = (uint8_t)(cm4_iq >> 8);
    tx_message.Data[6] = (uint8_t)cm4_iq;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);	

		CAN2_TX_Wait();
}

void Get_Track_Speed(CAN_TypeDef *CANx)
{
    CanTxMsg tx_message;	
	
		tx_message.StdId = 0x01;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;    
    tx_message.Data[0] = 0x00;
    tx_message.Data[1] = 0x0d;
    tx_message.Data[2] = 0x00;
    tx_message.Data[3] = 0x00;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
	
		CAN2_TX_Wait();
		
		tx_message.StdId = 0x02;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;    
    tx_message.Data[0] = 0x00;
    tx_message.Data[1] = 0x0d;
    tx_message.Data[2] = 0x00;
    tx_message.Data[3] = 0x00;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
		
		CAN2_TX_Wait();
		
		tx_message.StdId = 0x03;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;    
    tx_message.Data[0] = 0x00;
    tx_message.Data[1] = 0x0d;
    tx_message.Data[2] = 0x00;
    tx_message.Data[3] = 0x00;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
		
		CAN2_TX_Wait();
		
		tx_message.StdId = 0x04;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;    
    tx_message.Data[0] = 0x00;
    tx_message.Data[1] = 0x0d;
    tx_message.Data[2] = 0x00;
    tx_message.Data[3] = 0x00;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
		
		CAN2_TX_Wait();
}

void Set_Leg_Speed(CAN_TypeDef *CANx, int32_t Front_Leg, int32_t Back_Leg)
{
    CanTxMsg tx_message;
	
	  tx_message.StdId = 0x07;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = 0x01;
    tx_message.Data[1] = 0xF4;
    tx_message.Data[2] = 0x05;
    tx_message.Data[3] = (uint8_t)(Front_Leg >> 24);
    tx_message.Data[4] = (uint8_t)(Front_Leg >> 16);
    tx_message.Data[5] = (uint8_t)(Front_Leg >> 8);
    tx_message.Data[6] = (uint8_t)Front_Leg;
    tx_message.Data[7] = 0x00;	
    CAN_Transmit(CANx,&tx_message);
	
		CAN2_TX_Wait();
	
	  tx_message.StdId = 0x08;
	  tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
		
	  tx_message.Data[0] = 0x01;
    tx_message.Data[1] = 0xF4;
    tx_message.Data[2] = 0x05;
    tx_message.Data[3] = (uint8_t)(Back_Leg >> 24);
    tx_message.Data[4] = (uint8_t)(Back_Leg >> 16);
    tx_message.Data[5] = (uint8_t)(Back_Leg >> 8);
    tx_message.Data[6] = (uint8_t)Back_Leg;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);	

		CAN2_TX_Wait();
}


void Enable_Balance_Driver(CAN_TypeDef* CANx)
{
	CanTxMsg tx_message;
	u8 i=0;

	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;

	for(i=0;i<2;i++)
	{
		tx_message.StdId = i+5;	//0x621
		tx_message.DLC = 0x08;
		
		tx_message.Data[0] = 0x00;
		tx_message.Data[1] = 0x07;
		tx_message.Data[2] = 0x05;
		tx_message.Data[3] = 0x00;
		tx_message.Data[4] = 0x00;
		tx_message.Data[5] = 0x00;
		tx_message.Data[6] = 0x01;
		tx_message.Data[7] = 0x00;
		CAN_Transmit(CANx,&tx_message);
		
		CAN2_TX_Wait();
		delay_ms(1);
	}
}

void Disable_Balance_Driver(CAN_TypeDef* CANx)
{
	CanTxMsg tx_message;
	u8 i=0;

	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;

	for(i=0;i<2;i++)
	{
		tx_message.StdId = i+5;	//0x621
		tx_message.DLC = 0x08;
		
		tx_message.Data[0] = 0x00;
		tx_message.Data[1] = 0x07;
		tx_message.Data[2] = 0x05;
		tx_message.Data[3] = 0x00;
		tx_message.Data[4] = 0x00;
		tx_message.Data[5] = 0x00;
		tx_message.Data[6] = 0x00;
		tx_message.Data[7] = 0x00;
		CAN_Transmit(CANx,&tx_message);
		
		CAN2_TX_Wait();
		delay_ms(1);
	}
}

void Enable_Track_Driver(CAN_TypeDef* CANx)
{
	CanTxMsg tx_message;
	u8 i=0;

	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;

	for(i=0;i<4;i++)
	{
		tx_message.StdId = i+1;	//0x621
		tx_message.DLC = 0x08;
		
		tx_message.Data[0] = 0x00;
		tx_message.Data[1] = 0x07;
		tx_message.Data[2] = 0x05;
		tx_message.Data[3] = 0x00;
		tx_message.Data[4] = 0x00;
		tx_message.Data[5] = 0x00;
		tx_message.Data[6] = 0x01;
		tx_message.Data[7] = 0x00;
		CAN_Transmit(CANx,&tx_message);
		
		CAN2_TX_Wait();
		
		delay_ms(1);
	}
}

void Disable_Track_Driver(CAN_TypeDef* CANx)
{
	CanTxMsg tx_message;
	u8 i=0;

	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;

	for(i=0;i<4;i++)
	{
		tx_message.StdId = i+1;	//0x621
		tx_message.DLC = 0x08;
		
		tx_message.Data[0] = 0x00;
		tx_message.Data[1] = 0x07;
		tx_message.Data[2] = 0x05;
		tx_message.Data[3] = 0x00;
		tx_message.Data[4] = 0x00;
		tx_message.Data[5] = 0x00;
		tx_message.Data[6] = 0x00;
		tx_message.Data[7] = 0x00;
		CAN_Transmit(CANx,&tx_message);
		
		CAN2_TX_Wait();
		delay_ms(1);
	}
}

void Enable_Leg_Driver(CAN_TypeDef* CANx)
{
	CanTxMsg tx_message;
	u8 i=0;

	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;

	for(i=0;i<2;i++)
	{
		tx_message.StdId = i+7;	//0x621
		tx_message.DLC = 0x08;
		
		tx_message.Data[0] = 0x00;
		tx_message.Data[1] = 0x07;
		tx_message.Data[2] = 0x05;
		tx_message.Data[3] = 0x00;
		tx_message.Data[4] = 0x00;
		tx_message.Data[5] = 0x00;
		tx_message.Data[6] = 0x01;
		tx_message.Data[7] = 0x00;
		CAN_Transmit(CANx,&tx_message);
		
		CAN2_TX_Wait();
		delay_ms(1);
	}
}

/********************************************************************************
   给电调板发送指令，ID号为0x1FF，只用两个电调板，数据回传ID为0x205和0x206
	 cyq:更改为发送三个电调的指令。
*********************************************************************************/
