#include "main.h"
#include "stdio.h"



unsigned int can1_tx_success_flag;

static void CAN_Delay_Us(unsigned int t)
{
	int i;
	for(i=0;i<t;i++)
	{
		int a=40;
		while(a--);
	}
}

void CAN1_TX_Wait(void)
{
	u8 CAN_Time_Out = 0;
	can1_tx_success_flag = 0;
	while(can1_tx_success_flag == 0)
	{
			CAN_Delay_Us(1);
			CAN_Time_Out++;
			if(CAN_Time_Out>250)
			{
					break;
			}
	}
}

int my_abs(int x)
{
	if(x<0) x = -x;
	
	return x;
}

float my_fabs(float x)
{
	if(x<0) x = -x;
	
	return x;
}

extern u8 Push_Motor_Limit_Switch_Value;
extern int32_t Push_Motor_position;

void Limit_Switch_process(CanRxMsg * msg)
{
	Push_Motor_Limit_Switch_Value = msg->Data[0];
}

extern Arm_Status_Data Arm;

void Can1ReceiveMsgProcess(CanRxMsg * msg)
{      
	
	switch(msg->StdId)
	{			
		case 0x09B:    // push motor Limit_Switch 
		{
			Push_Motor_position = ((msg->Data[4]<<24)|(msg->Data[5]<<16)|(msg->Data[6]<<8)|(msg->Data[7]));
		}break;
		case 0x09C:    // push motor Limit_Switch 
		{
			Push_Motor_Limit_Switch_Value = msg->Data[0];
		}break;
		
		case 0x01B:    //
		{
			Arm.Joint_1_Position = ((msg->Data[4]<<24)|(msg->Data[5]<<16)|(msg->Data[6]<<8)|(msg->Data[7]));
		}break;
		
		case 0x02B:    //
		{
			Arm.Joint_2_Position = -((msg->Data[4]<<24)|(msg->Data[5]<<16)|(msg->Data[6]<<8)|(msg->Data[7]));
		}break;
		
		case 0x03B:    //
		{
			Arm.Joint_3_Position = ((msg->Data[4]<<24)|(msg->Data[5]<<16)|(msg->Data[6]<<8)|(msg->Data[7]));
		}break;
		
		case 0x04B:    //
		{
			Arm.Joint_4_Position = ((msg->Data[4]<<24)|(msg->Data[5]<<16)|(msg->Data[6]<<8)|(msg->Data[7]));
		}break;
		
		case 0x01C:    //
		{
			Arm.Joint_1_Limit_Switch = ((msg->Data[1]<<1)|(msg->Data[0]));
		}break;
		
		case 0x02C:    //
		{
			Arm.Joint_2_Limit_Switch = ((msg->Data[1]<<1)|(msg->Data[0]));
		}break;
		
		case 0x03C:    //
		{
			Arm.Joint_3_Limit_Switch = ((msg->Data[1]<<1)|(msg->Data[0]));
		}break;
		
		case 0x04C:    //
		{
			Arm.Joint_4_Limit_Switch = ((msg->Data[1]<<1)|(msg->Data[0]));
		}break;
		
		default:
		{
		}
	}
}

/****************************************************************************************
                                       复位指令
Group   取值范围 0-7
Number  取值范围 0-15，其中Number==0时，为广播发送
*****************************************************************************************/
void CAN_RoboModule_DRV_Reset(unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x000;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    tx_message.Data[0] = 0x55;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN1_TX_Wait();
}


/****************************************************************************************
                                     模式选择指令
Group   取值范围 0-7
Number  取值范围 0-15，其中Number==0时，为广播发送

Mode    取值范围

OpenLoop_Mode                       0x01
Current_Mode                        0x02
Velocity_Mode                       0x03
Position_Mode                       0x04
Velocity_Position_Mode              0x05
Current_Velocity_Mode               0x06
Current_Position_Mode               0x07
Current_Velocity_Position_Mode      0x08
*****************************************************************************************/
void CAN_RoboModule_DRV_Mode_Choice(unsigned char Group,unsigned char Number,unsigned char Mode)
{
    unsigned short can_id = 0x001;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    tx_message.Data[0] = Mode;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN1_TX_Wait();
}

/****************************************************************************************
                                   开环模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = ±5000时，最大输出电压为电源电压

*****************************************************************************************/
void CAN_RoboModule_DRV_OpenLoop_Mode(unsigned char Group,unsigned char Number,short Temp_PWM)
{
    unsigned short can_id = 0x002;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    CAN_Transmit(CAN1,&tx_message);
    
   CAN1_TX_Wait();
}

/****************************************************************************************
                                   电流模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_current的取值范围如下：
-32768 ~ +32767，单位mA

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Current)
{
    unsigned short can_id = 0x003;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = my_abs(Temp_PWM);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    CAN_Transmit(CAN1,&tx_message);
		
		CAN1_TX_Wait();
   
}

/****************************************************************************************
                                   速度模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_velocity的取值范围如下：
-32768 ~ +32767，单位RPM

*****************************************************************************************/
void CAN_RoboModule_DRV_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity)
{
    unsigned short can_id = 0x004;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = -(Temp_PWM);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN1_TX_Wait();
}

/****************************************************************************************
                                   位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc

#define UP_POSITION       0X1E848   // 125000
#define MIDDLE_POSITION   0X0FDE8   // 65000
#define DOWN_POSITION     0X01388   // 5000

#define FULL_PWM          0X01388   // 5000


*****************************************************************************************/
void CAN_RoboModule_DRV_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,long Temp_Position)
{
    unsigned short can_id = 0x005;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = -(Temp_PWM);
    }
    
		
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);
    CAN_Transmit(CAN1,&tx_message);
    
    CAN1_TX_Wait();
}

/****************************************************************************************
                                      配置指令
Temp_Time1的取值范围: 0 ~ 255，为0时候，为关闭电流速度位置反馈功能
Temp_Time2的取值范围: 0 ~ 255，为0时候，为关闭限位信号反馈功能
*****************************************************************************************/
void CAN_RoboModule_DRV_Config(unsigned char Group,unsigned char Number,unsigned char Temp_Time1,unsigned char Temp_Time2)
{
    unsigned short can_id = 0x00A;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;
    
    tx_message.Data[0] = Temp_Time1;
    tx_message.Data[1] = Temp_Time2;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN1_TX_Wait();
}

void Arm_Motor_Driver_Init(void)
{
	CAN_RoboModule_DRV_Reset(0,0);                      //对0组所有驱动器进行复位 
	delay_ms(300);	
  CAN_RoboModule_DRV_Mode_Choice(0,1,Velocity_Mode);  //0组的1驱动器 进入Velocity_Mode,Arm_joint1
	CAN_RoboModule_DRV_Mode_Choice(0,2,Velocity_Mode);  //0组的2驱动器 进入Velocity_Mode,Arm_joint2
	CAN_RoboModule_DRV_Mode_Choice(0,3,Velocity_Mode);  //0组的3驱动器 进入Velocity_Mode,Arm_joint3
	CAN_RoboModule_DRV_Mode_Choice(0,4,Velocity_Mode);  //0组的4驱动器 进入Velocity_Mode,Arm_joint4
	CAN_RoboModule_DRV_Mode_Choice(0,5,OpenLoop_Mode);  //0组的5驱动器 进入OpenLoop_Mode,Arm_joint5
	//CAN_RoboModule_DRV_Mode_Choice(0,6,Current_Mode);  //0组的6驱动器 进入Current_Mode,Arm_joint6
	CAN_RoboModule_DRV_Mode_Choice(0,6,OpenLoop_Mode);  //0组的6驱动器 进入Current_Mode,Arm_joint6
	CAN_RoboModule_DRV_Mode_Choice(0,7,OpenLoop_Mode);  //0组的7驱动器 进入OpenLoop_Mode,camera_plat
	CAN_RoboModule_DRV_Mode_Choice(0,8,OpenLoop_Mode);  //0组的8驱动器 进入OpenLoop_Mode,camera_plat
	CAN_RoboModule_DRV_Mode_Choice(0,9,Position_Mode);  //0组的9驱动器 进入Position_Mode,push_motor
	delay_ms(300);	
	CAN_RoboModule_DRV_Config(0,1,20,20);
	CAN_RoboModule_DRV_Config(0,2,20,20);
	CAN_RoboModule_DRV_Config(0,3,20,20);
	CAN_RoboModule_DRV_Config(0,4,20,20);
	CAN_RoboModule_DRV_Config(0,5,0,0);
	CAN_RoboModule_DRV_Config(0,6,0,0);
	CAN_RoboModule_DRV_Config(0,7,0,0);
	CAN_RoboModule_DRV_Config(0,8,0,0);
	CAN_RoboModule_DRV_Config(0,9,50,50);
	delay_ms(200);
}

void Robo_Module_Speed_Set(uint8_t node_no,int Temp_Velocity) 
{
	int Temp_PWM = 5000;
	
	if(node_no == 1)
	{
		CAN_RoboModule_DRV_Velocity_Mode(0,1,Temp_PWM,Temp_Velocity);
	}
	else if(node_no == 2)
	{
		CAN_RoboModule_DRV_Velocity_Mode(0,2,Temp_PWM,Temp_Velocity);
	}	
	else if(node_no == 3)
	{
		CAN_RoboModule_DRV_Velocity_Mode(0,3,Temp_PWM,Temp_Velocity);
	}	
	else if(node_no == 4)
	{
		CAN_RoboModule_DRV_Velocity_Mode(0,4,Temp_PWM,Temp_Velocity);
	}	
	else if(node_no == 5)
	{
		CAN_RoboModule_DRV_OpenLoop_Mode(0,5,Temp_Velocity);
	}	
	else if(node_no == 6)
	{
		//CAN_RoboModule_DRV_OpenLoop_Mode(0,6,Temp_Velocity);
		//CAN_RoboModule_DRV_Current_Mode(0,6,Temp_PWM,Temp_Velocity);
	}	
	else if(node_no == 7)
	{
		CAN_RoboModule_DRV_OpenLoop_Mode(0,7,Temp_Velocity);
	}	
	else if(node_no == 8)
	{
		CAN_RoboModule_DRV_OpenLoop_Mode(0,8,Temp_Velocity);
	}	
		
}

void Robo_Module_Current_Set(uint8_t node_no,int Temp_PWM,int Temp_Current) 
{
	CAN_RoboModule_DRV_Current_Mode(0,6,Temp_PWM,Temp_Current);
}