#include <stm32f4xx.h>
#include "main.h"

extern volatile Arm_Rocker_Data Arm_Rocker;

extern uint16_t remote_channel_data[20];
extern u8 SIYI_Remote_data_buf[30];
extern u8 SIYI_Remote_Data_Flag;
extern int RC[20];

extern u8 Pad_App_Data_Buf[40];
extern u8 Pad_App_Data_Flag;

extern u8 Control_Platform_Data_Buf[40];
extern u8 Control_Platform_Data_Flag;

extern int Yaw_Speed,Ptich_Speed;

extern u8 Robot_Move_Mode,Robot_Move_Mode_Last;
extern int Robot_Line_Speed,Robot_Rotate_Speed;
extern  u8 Robot_Move_Enable;
extern int Front_Leg_speed,Back_Leg_speed;
extern u8 Robot_Posture_Reset_Flag,Robot_Arm_Reset_Flag;
extern u8 Robot_leg_control_Flag;
extern u8 Robot_wheel_height_control_Flag;
extern u8 Camera_Platform_Control_Flag;
extern u8 Robot_Arm_Control_Flag;
extern int32_t  Temp_wheel_position;
extern u8 Push_Motor_Limit_Switch_Value;

Pad_App_Control_Data Pad_control_Data; //

extern u8 ROS_Control_Data_Buf[30];
extern u8 ROS_Control_Data_Flag;

extern union ROS_Data_Receive  //
{
    float float_type;
    unsigned char char_type[4];
}Target_Line_Speed,Target_Angl_Speed,Target_Position_X,Target_Position_Y,Target_Angle,Target_left_right_speed;     //


int error_imu,error_remote;

void SIYI_Remote_Data_Enable(void)
{
	static u8 remote_data_count=0;
	int i=0;
	
	if(SIYI_Remote_Data_Flag==1)
	{
		siyi_remote_data_process(SIYI_Remote_data_buf);  // data cycle : 20ms
		remote_data_count=0;
		SIYI_Remote_Data_Flag=0;
	}
	
	if(remote_data_count++>100)
	{
		remote_data_count=100;
		for(i=0;i<16;i++)
		{
			RC[i] = 0;
		}
		error_remote++;
	}
	
	for(i=0;i<16;i++)
	{
		if(RC[i]<-40) RC[i] += 40;
		else if(RC[i]<40) RC[i] = 0;
		else RC[i] -= 40;
	}
	
	////--------------------------------------------////
	
	if(RC[10]>500)
	{
		Robot_Move_Enable = 1;	
	}
	else
	{
		Robot_Move_Enable = 0;
	}
	
	Robot_Line_Speed = 1.0*RC[1]/6.25;
	Robot_Rotate_Speed = 1.0*RC[3]/6.25;
	
	////--------------------------------------------////
	
	if(RC[7]>500)         // Robot_Move_Mode  choose
	{
		Robot_Move_Mode = 3;
	}
	else if(RC[7]>-500)
	{
		Robot_Move_Mode = 2;
	}
	else 
	{
		Robot_Move_Mode = 1;
	}
	
	////--------------------------------------------////
	
	if(RC[5]>500)        //  Robot_wheel   Robot_leg   choose
	{
		Robot_wheel_height_control_Flag = 1;
		Robot_leg_control_Flag = 0;
	}
	else if(RC[5]>-500)
	{
		Robot_wheel_height_control_Flag = 0;
		if(RC[6]>500)
		{
			Robot_leg_control_Flag = 3;
		}
		else if(RC[6]>-500)
		{
			Robot_leg_control_Flag = 2;
		}
		else 
		{
			Robot_leg_control_Flag = 1;
		}
	}
	else 
	{
		Robot_wheel_height_control_Flag = 0;
		Robot_leg_control_Flag = 0;
	}
	
	if(Robot_Move_Mode==1)
	{
		if(Robot_leg_control_Flag==2)
		{
			Front_Leg_speed = -RC[1]*3;
			Back_Leg_speed = RC[1]*3;	
		}
		else if(Robot_leg_control_Flag==3)
		{
			Front_Leg_speed = -RC[1]*3;
			Back_Leg_speed = 0;	
		}
		else if(Robot_leg_control_Flag==1)
		{
			Front_Leg_speed = 0;
			Back_Leg_speed = RC[1]*3;	
		}
		else
		{
			Front_Leg_speed = 0;
			Back_Leg_speed = 0;	
		}
		
	}
	else
	{
		Front_Leg_speed = 0;
		Back_Leg_speed = 0;	
	}
	
	////--------------------------------------------////
	
//	if(RC[6]>500) 
//	{
//		Robot_Posture_Reset_Flag = 3;
//	}
//	else if(RC[6]>-500)
//	{
//		Robot_Posture_Reset_Flag = 2;
//	}
//	else 
//	{
//		Robot_Posture_Reset_Flag = 1;
//	}
	
	
	if(RC[4]>500)           //    Robot_leg_control_Flag
	{
		Robot_Arm_Control_Flag = 1;
		Camera_Platform_Control_Flag = 0;
	}
	else if(RC[4]>-500)
	{
		Robot_Arm_Control_Flag = 0;
		Camera_Platform_Control_Flag = 1;
	}
	else 
	{
		Robot_Arm_Control_Flag = 0;
		Camera_Platform_Control_Flag = 0;
	}
	
	////--------------------------------------------////
	
	Arm_Rocker_Data_Process();
	
	if(Camera_Platform_Control_Flag==1) 
	{
		Yaw_Speed = -RC[0]*5;
		Ptich_Speed = -RC[1]*5;
	}
	else
	{
		Yaw_Speed = 0;
		Ptich_Speed = 0;
	}	
	
	////---------------------------------------------/////

}

void Arm_Rocker_Data_Process(void) 
{
	if(Robot_Arm_Control_Flag==1)
	{
		if(RC[11]>0)
		{
			if(RC[12]>0)
			{
				Arm_Rocker.Key1=1;
				Arm_Rocker.Key2=1;
			}
			else
			{
				Arm_Rocker.Key1=1;
				Arm_Rocker.Key2=0;
			}
		}
		else
		{
			if(RC[12]>0)
			{
				Arm_Rocker.Key1=0;
				Arm_Rocker.Key2=1;
			}
			else
			{
				Arm_Rocker.Key1=0;
				Arm_Rocker.Key2=0;
			}
		}
		
		if(RC[14]>0)
		{
			Arm_Rocker.Key3=1;
		}
		else
		{
			Arm_Rocker.Key3=0;
		}
		
		if(RC[15]>0)
		{
			Arm_Rocker.Key4=1;
		}
		else
		{
			Arm_Rocker.Key4=0;
		}
		
		if(RC[10]>0)
		{
			Arm_Rocker.Key5=1;
		}
		else
		{
			Arm_Rocker.Key5=0;
		}
		
		if(RC[13]>0)
		{
			Arm_Rocker.Key6=1;
		}
		else
		{
			Arm_Rocker.Key6=0;
		}
		
		
		Arm_Rocker.X = -1.0*RC[0]/6.25; // 800/128=6.25
		Arm_Rocker.Y = -1.0*RC[1]/6.25;
		Arm_Rocker.Z1 = 1.0*RC[2]/6.25;
		Arm_Rocker.Z2 = 1.0*RC[3]/6.25;	
		
		Arm_Rocker.Speed_Adjust = my_fabs(1.0*RC[9]/6.25)*2;
		
//		if(Arm_Rocker.X>-5&&Arm_Rocker.X<5) Arm_Rocker.X=0;
//		if(Arm_Rocker.Y>-5&&Arm_Rocker.Y<5) Arm_Rocker.Y=0;
//		if(Arm_Rocker.Z1>-5&&Arm_Rocker.Z1<5) Arm_Rocker.Z1=0;
//		if(Arm_Rocker.Z2>-5&&Arm_Rocker.Z2<5) Arm_Rocker.Z2=0;
	}
	else
	{		
		Arm_Rocker.Key1=0;
		Arm_Rocker.Key2=0;
		Arm_Rocker.Key3=0;
		Arm_Rocker.Key4=0;
		Arm_Rocker.Key5=0;
		Arm_Rocker.Key6=0;
		Arm_Rocker.X = 0;
		Arm_Rocker.Y = 0;
		Arm_Rocker.Z1 = 0;
		Arm_Rocker.Z2 = 0;	
		Arm_Rocker.Speed_Adjust =0;
	}
}

void Pad_Control_Data_Process(void)
{
	static u8 Data_count;
	
	if(Pad_App_Data_Flag==1)
	{
		Pad_control_Data.Control_Mode = Pad_App_Data_Buf[1];
		Pad_control_Data.Control_Object = Pad_App_Data_Buf[2];
		if(Pad_control_Data.Control_Mode==0x01 && Pad_control_Data.Control_Object==0x01)
		{
			Pad_control_Data.Rocker_X = 0;
			Pad_control_Data.Rocker_Y = Pad_App_Data_Buf[5]-0X80;
		}
		else
		if(Pad_control_Data.Control_Mode==0x01 && Pad_control_Data.Control_Object==0x02)
		{
			Pad_control_Data.Rocker_X = Pad_App_Data_Buf[4]-0X80;
			Pad_control_Data.Rocker_Y = 0;
		}
		else
		if(Pad_control_Data.Control_Mode==0x01 && (Pad_control_Data.Control_Object==0x03 || Pad_control_Data.Control_Object==0x04))
		{
			Pad_control_Data.Rocker_X = Pad_App_Data_Buf[4]-0X80;
			Pad_control_Data.Rocker_Y = Pad_App_Data_Buf[5]-0X80;
		}
		else
		{
			Pad_control_Data.Rocker_X = 0;
			Pad_control_Data.Rocker_Y = 0;
		}
		
		
		Pad_App_Data_Flag=0;
		Data_count=0;
	}
	
	if(Data_count++>100)
	{
		Data_count=100;
		
		Pad_control_Data.Control_Mode = 0;
		Pad_control_Data.Arm_Sig_Joint_Control_Object = 0;
		Pad_control_Data.Rocker_X = 0;
		Pad_control_Data.Rocker_Y = 0;
	}
}


void ROS_Control_Data_Process(void)
{
	char i=0;
	if(ROS_Control_Data_Flag==1)
	{
		for(i=0;i<4;i++)
		{
			Target_Line_Speed.char_type[i] = ROS_Control_Data_Buf[i];
			Target_Angl_Speed.char_type[i] = ROS_Control_Data_Buf[i+4];
			Target_Position_X.char_type[i] = ROS_Control_Data_Buf[i+8];
			Target_Position_Y.char_type[i] = ROS_Control_Data_Buf[i+12];
			Target_Angle.char_type[i] = ROS_Control_Data_Buf[i+16];
			Target_left_right_speed.char_type[i] = ROS_Control_Data_Buf[i+16];
		}
		ROS_Control_Data_Flag = 0;
	}
}

u8 Data_Filter(u8 data)
{
	static u8 data_buf[15],data_count=0;
	u8 Filter_Len=10,i=0,j=0,compare=0,compare_count=0;
	
	data_buf[data_count++] = data;
	if(data_count>=Filter_Len) data_count=0;
	
	for(i=0;i<Filter_Len;i++)
	{
		compare = data_buf[i];
		compare_count=0;
		for(j=0;j<Filter_Len;j++)
		{
			if(compare==data_buf[j])
			{
				compare_count++;
				if(compare_count>5) return compare;
			}				
		}
	}
	return 0;
}

void Control_Platform_Data_Enable(void)
{
	static u8 Data_count=0;	
	static int count=0;
	
	int zero_limit=10;
	
	if(Control_Platform_Data_Flag==1)
	{
		Arm_Rocker.Key1 = Control_Platform_Data_Buf[1]&0X01;
		Arm_Rocker.Key2 = (Control_Platform_Data_Buf[1]&0X02)>>1;
		Arm_Rocker.Key3 = (Control_Platform_Data_Buf[0]&0X10)>>4;
		Arm_Rocker.Key4 = (Control_Platform_Data_Buf[0]&0X20)>>5;
		Arm_Rocker.Key5 = (Control_Platform_Data_Buf[0]&0X80)>>7;	
		Arm_Rocker.Key6 = (Control_Platform_Data_Buf[0]&0X40)>>6;
		
		Arm_Rocker.X = 0X80 - Control_Platform_Data_Buf[4];
		Arm_Rocker.Y = 0x80 - Control_Platform_Data_Buf[5];
		Arm_Rocker.Z1 = Control_Platform_Data_Buf[7] - 0X80;
		Arm_Rocker.Z2 = Control_Platform_Data_Buf[6] - 0X80;
		
		if(Arm_Rocker.X<-zero_limit) Arm_Rocker.X += zero_limit;
		else if(Arm_Rocker.X<zero_limit) Arm_Rocker.X = 0;
		else Arm_Rocker.X -= zero_limit;
		
		if(Arm_Rocker.Y<-zero_limit) Arm_Rocker.Y += zero_limit;
		else if(Arm_Rocker.Y<zero_limit) Arm_Rocker.Y = 0;
		else Arm_Rocker.Y -= zero_limit;
		
		if(Arm_Rocker.Z1<-zero_limit) Arm_Rocker.Z1 += zero_limit;
		else if(Arm_Rocker.Z1<zero_limit) Arm_Rocker.Z1 = 0;
		else Arm_Rocker.Z1 -= zero_limit;
		
		if(Arm_Rocker.Z2<-zero_limit) Arm_Rocker.Z2 += zero_limit;
		else if(Arm_Rocker.Z2<zero_limit) Arm_Rocker.Z2 = 0;
		else Arm_Rocker.Z2 -= zero_limit;
		
		Arm_Rocker.Speed_Adjust = 0XFF - Control_Platform_Data_Buf[10];
		if(my_abs(Arm_Rocker.Speed_Adjust)<10) Arm_Rocker.Speed_Adjust=0;
		
		Robot_Line_Speed = -(Control_Platform_Data_Buf[8] - 0x80);
		Robot_Rotate_Speed = -(Control_Platform_Data_Buf[9] - 0x80);
		Robot_Move_Enable = (Control_Platform_Data_Buf[1]&0X10)>>4;
		
		if(Robot_Line_Speed<-zero_limit) Robot_Line_Speed += zero_limit;
		else if(Robot_Line_Speed<zero_limit) Robot_Line_Speed = 0;
		else Robot_Line_Speed -= zero_limit;
		
		if(Robot_Rotate_Speed<-zero_limit) Robot_Rotate_Speed += zero_limit;
		else if(Robot_Rotate_Speed<zero_limit) Robot_Rotate_Speed = 0;
		else Robot_Rotate_Speed -= zero_limit;
		
		////---------------------------------------------------////
		
		if((Control_Platform_Data_Buf[1]&0X80)==0x80)
		{
			Robot_Move_Mode =3;
		}
		else if((Control_Platform_Data_Buf[1]&0X40)==0x40)
		{
			Robot_Move_Mode =2;
		}
		else if((Control_Platform_Data_Buf[1]&0X20)==0x20)
		{
			Robot_Move_Mode =1;
		}
		else
		{
			Robot_Move_Mode =0;
		}		
		
		if((Control_Platform_Data_Buf[2]&0X10)==0x10)
		{
			Robot_Arm_Reset_Flag =1;
		}
		else
		{
			Robot_Arm_Reset_Flag =0;
		}		
		
		if((Control_Platform_Data_Buf[2]&0X20)==0x20)
		{
			Robot_Posture_Reset_Flag =1;
		}
		else
		{
			Robot_Posture_Reset_Flag =0;
		}		
		
		////---------------------------------------------------////
		
		if((Control_Platform_Data_Buf[0]&0X01)==0x01)
		{
			Front_Leg_speed = -10*Arm_Rocker.Speed_Adjust;
		}
		else if((Control_Platform_Data_Buf[0]&0X02)==0x02)
		{
			Front_Leg_speed = 10*Arm_Rocker.Speed_Adjust;
		}
		else
		{
			Front_Leg_speed = 0;
		}
		
		if((Control_Platform_Data_Buf[0]&0X04)==0x04)
		{
			Back_Leg_speed = 10*Arm_Rocker.Speed_Adjust;
		}
		else if((Control_Platform_Data_Buf[0]&0X08)==0x08)
		{
			Back_Leg_speed = -10*Arm_Rocker.Speed_Adjust;
		}
		else
		{
			Back_Leg_speed = 0;
		}
		
		////---------------------------------------------------////
		
		if((Control_Platform_Data_Buf[2]&0X01)==1)
		{
			Ptich_Speed = SPEED_SCALE*Arm_Rocker.Speed_Adjust;
		}
		else if((Control_Platform_Data_Buf[2]&0X02)==0x02)
		{
			Ptich_Speed = -SPEED_SCALE*Arm_Rocker.Speed_Adjust;
		}
		else
		{
			Ptich_Speed = 0;
		}
		
		if((Control_Platform_Data_Buf[2]&0X04)==0x04)
		{
			Yaw_Speed = SPEED_SCALE*Arm_Rocker.Speed_Adjust;
		}
		else if((Control_Platform_Data_Buf[2]&0X08)==0x08)
		{
			Yaw_Speed = -SPEED_SCALE*Arm_Rocker.Speed_Adjust;
		}
		else
		{
			Yaw_Speed = 0;
		}
		
//		if(count++>5)
//		{
//			count=0;
//			//Up_Load_Robot_Data(); //
//			printf("--%d , %d , %d , %d , %d , %d ",Arm_Rocker.Key1,Arm_Rocker.Key2,Arm_Rocker.Key3,Arm_Rocker.Key4,Arm_Rocker.Key5,Arm_Rocker.Key6);
//			printf("--%d , %d , %d , %d , %d ",Arm_Rocker.X,Arm_Rocker.Y,Arm_Rocker.Z1,Arm_Rocker.Z2,Arm_Rocker.Speed_Adjust);
//			printf("--%d , %d , %d , %d \n",Robot_Move_Mode,Robot_Move_Enable,Robot_Line_Speed,Robot_Rotate_Speed);
//			
//			printf("--%d , %d , %d , %d , %d , %d \n",Front_Leg_speed,Back_Leg_speed,Yaw_Speed,Ptich_Speed,Robot_Arm_Reset_Flag,Robot_Posture_Reset_Flag);
//		}
		//Set_Leg_Speed(CAN2,Front_Leg_speed,Back_Leg_speed);
		
		Control_Platform_Data_Flag=0;
		Data_count=0;
	}
	
	if(Data_count++>200)
	{
		Data_count=201;
		
		Robot_Move_Mode =0;
		
		Arm_Rocker.Key1 = 0;
		Arm_Rocker.Key2 = 0;
		Arm_Rocker.Key3 = 0;
		Arm_Rocker.Key4 = 0;
		Arm_Rocker.Key5 = 0;	
		Arm_Rocker.Key6 = 0;
		
		Arm_Rocker.X = 0;
		Arm_Rocker.Y = 0;
		Arm_Rocker.Z1 = 0;
		Arm_Rocker.Z2 = 0;
		Arm_Rocker.Speed_Adjust = 0;
		
		Front_Leg_speed = 0;
		Back_Leg_speed = 0;
		Yaw_Speed = 0;
		Ptich_Speed = 0;
		
		Robot_Line_Speed = 0;
		Robot_Rotate_Speed = 0;
		Robot_Move_Enable = 0;
		
	}
	
}

void Remote_Data_Enable(void)
{
	static u8 choose=0;
	
	Robot_Move_Mode_Last = Robot_Move_Mode;
	
	if(Control_Platform_Data_Flag==1)
	{
		choose = 1;
	}
	
	if(choose==1)
	{
		Control_Platform_Data_Enable();
	}
	else 
	{
		SIYI_Remote_Data_Enable();
	}
	
	if(Robot_Move_Mode_Last!=2 && Robot_Move_Mode==2)
	{
//		Enable_Track_Driver(CAN2);
//		Enable_Track_Driver(CAN2);
	}
	if(Robot_Move_Mode_Last==2 && Robot_Move_Mode!=2)
	{
		//Disable_Track_Driver(CAN2);
		//Disable_Track_Driver(CAN2);
	}
	
}
