#include "main.h"

////-------------------------------------------////

extern Arm_Status_Data Arm;
extern volatile Arm_Rocker_Data Arm_Rocker; 

extern u8 Camera_Platform_Control_Flag;
extern int RC[16];
extern u8 Robot_Posture_Reset_Flag,Robot_Arm_Reset_Flag;

double Arm_Target_Speed_X,Arm_Target_Speed_Y;
double Theta_1,Theta_2,Theta_3,Theta_4;
double d_Theta_1,d_Theta_2,d_Theta_3,d_Theta_4;
double Arm_X_Offset,Arm_Y_Offset,Arm_X_Offset_Last,Arm_Y_Offset_Last;
double Arm_Target_X,Arm_Target_Y;
double Arm_Reset_X,Arm_Reset_Y;
double Arm_Kp=1.0,Arm_Kd=1.8;
double Target_Joint_4_Angle;

double Joint_2_Reset_Angle=-84.5,Joint_3_Reset_Angle=151.6,Joint_4_Reset_Angle=-113.4;

double Joint_2_Real_Reset_Angle=0,Joint_3_Real_Reset_Angle=90,Joint_4_Real_Reset_Angle=0;

u8 Arm_Control_Mode;

int Yaw_Speed,Ptich_Speed;

u8 Arm_Auto_Grasp_Flag=0;
int Process_Flag_1=0,Process_Flag_2=0,Process_Flag_3=0,Process_Flag_4=0,Process_Flag_5=0,Process_Flag_6=0;
int grasp_time=0,open_grasp=0;

//float Preset_Point_1_up[7]=  {1 , -163.5 , -2.0 , 70.8 , 113.8 , 0};
//float Preset_Point_1_down[7]={2 , -163.5 , -2.4 , 88.8 , 99.1  , 1};
//float Preset_Point_2_up[7]=  {1 , -152.4 , 1.2 , 68.0 , 113.3 , 0};
//float Preset_Point_2_down[7]={2 , -152.4 , 1.3 , 85.9 , 98.4  , 1};

float Preset_Point_1_up[7]=  {1 , -163.6 , -2.6 , 73.2 , 112.9 , 1};
float Preset_Point_1_down[7]={2 , -163.6 , -4.5 , 90.6 , 98.2  , 0};
float Preset_Point_1_grasp_up[7]={1 , -163.6 , -2.6 , 60.0 , 112.9 , 0};
float Preset_Point_1_grasp_down[7]={1 , -162.8 , -3.8 , 82.8 , 104.2 , 0};

float Preset_Point_2_up[7]=  {1 , -151.8 , 1.4 , 67.9 , 113.0 , 1};
float Preset_Point_2_down[7]={2 , -151.8 , -1.0 , 88.8 , 95.1  , 0};
float Preset_Point_2_return[7]={1 , -151.8 , -1.0 , 55.0 , 95.1  , 0};

float Grasp_Stretch_Point[7]={0 , 0.0 , 29.0 , 63.5 , 64 , 0};

////-------------------------------------------////

double A1,B1,A2,B2;

void Sig_Joint_Mov(void);

////-----------  获取轴位置  -------------------////

int Get_Joint_Motor_Pos(unsigned char num) //get arm axis pos (du)
{
	int pos;
	
	if(num == 1)
	{
		pos = Arm.Joint_1_Position;
	}
	else if(num == 2)
	{
		pos = Arm.Joint_2_Position;
	}
	else if(num == 3)
	{
		pos = Arm.Joint_3_Position;
	}
	else if(num == 4)
	{
		pos = Arm.Joint_4_Position;
	}
	
	return pos;
}

////-----------  获得实际角度 --------------------////

float Get_Joint_Angle(unsigned char num,int d)//get real angle of elbow (du)
{
	
	if(num==1)     //关节1
	{
	  return (1.0*d/(SCALE1*ENCRES*4)*360);//return (90-XI2_UP+(float)d*360/((float)SCALE1*ENCRES*4));
	}	
	else if(num==2)//关节2
	{		
	  //return (  + Joint_2_Reset_Angle + 1.0*d/(SCALE2*ENCRES*4)*360);
		return (Joint_2_Reset_Angle + 1.0*d/(SCALE2*ENCRES*4)*360);
	}
	else if(num==3)//关节3
	{
	  //return ( 90 - Joint_3_Reset_Angle - 1.0*d/(SCALE3*ENCRES*4)*360); 
	  return (Joint_3_Reset_Angle - 1.0*d/(SCALE3*ENCRES*4)*360); 
	}
	else if(num==4)//关节4
	{
	  //return ( + Joint_4_Reset_Angle + 1.0*d/(SCALE4*ENCRES*4)*360); 
		return (Joint_4_Reset_Angle + 1.0*d/(SCALE4*ENCRES*4)*360); 
	}
		
}

void Set_Arm_joint_Current(u8 node,int Temp_pwm,int Temp_current)
{
	Robo_Module_Current_Set(node,Temp_pwm,Temp_current);
}

void Set_Arm_Joint_Motor_Rotate_Speed(uint8_t Joint,int Rotate_Speed) 
{
	int Temp_PWM = 5000;
	
	if(Rotate_Speed> 6000) Rotate_Speed =  6000;
	if(Rotate_Speed<-6000) Rotate_Speed = -6000;
	
	switch(Joint)
	{			
		case 0x01:    // 
		{
			if((Arm.Joint_1_Limit_Switch==1)&&(Rotate_Speed<0))
			{
				Rotate_Speed = 0;
			}
			if((Arm.Joint_1_Limit_Switch==2)&&(Rotate_Speed>0))
			{
				Rotate_Speed = 0;
			}
			if(Arm.Joint_2_Limit_Switch==2 || Arm.Joint_2_Limit_Switch==1)
			{
				Rotate_Speed = 0;
			}
			if(Arm.Joint_2_Angle<-45+Joint_2_offset_Angle) 
			{
				Rotate_Speed = 0;
			}
			CAN_RoboModule_DRV_Velocity_Mode(0,1,Temp_PWM,Rotate_Speed);
		}break;
		
		case 0x02:    // 
		{
			if((Arm.Joint_2_Limit_Switch==2)&&(Rotate_Speed<0))
			{
				Rotate_Speed = 0;
			}
			if((Arm.Joint_2_Limit_Switch==1)&&(Rotate_Speed>0))
			{
				Rotate_Speed = 0;
			}
//			if((Arm.Joint_2_Angle<-90+Joint_2_Reset_Angle) && (Rotate_Speed>0)) 
//			{
//				Rotate_Speed = 0;
//			}
			if((Arm.Joint_2_Angle>90+Joint_2_offset_Angle) && (Rotate_Speed<0)) 
			{
				Rotate_Speed = 0;
			}
			CAN_RoboModule_DRV_Velocity_Mode(0,2,Temp_PWM,Rotate_Speed);
		}break;
		
		case 0x03:    // 
		{
			if((Arm.Joint_3_Limit_Switch==1)&&(Rotate_Speed<0))
			{
				Rotate_Speed = 0;
			}
			if((Arm.Joint_3_Limit_Switch==2)&&(Rotate_Speed>0))
			{
				Rotate_Speed = 0;
			}
			if((Arm.Joint_3_Angle<-Joint_3_offset_Angle) && (Rotate_Speed>0)) 
			{
				Rotate_Speed = 0;
			}
			CAN_RoboModule_DRV_Velocity_Mode(0,3,Temp_PWM,Rotate_Speed);
		}break;
		
		case 0x04:    // 
		{
			if((Arm.Joint_4_Limit_Switch==2)&&(Rotate_Speed<0))
			{
				Rotate_Speed = 0;
			}
			if((Arm.Joint_4_Limit_Switch==1)&&(Rotate_Speed>0))
			{
				Rotate_Speed = 0;
			}
			CAN_RoboModule_DRV_Velocity_Mode(0,4,Temp_PWM,Rotate_Speed);
		}break;
		
		case 0x05:    // 
		{
			CAN_RoboModule_DRV_OpenLoop_Mode(0,5,Rotate_Speed);
		}break;
		
		case 0x06:    // 
		{
			CAN_RoboModule_DRV_OpenLoop_Mode(0,6,Rotate_Speed);
		}break;
		
		case 0x07:    // 
		{
			CAN_RoboModule_DRV_OpenLoop_Mode(0,7,Rotate_Speed);
		}break;
		
		case 0x08:    // 
		{
			CAN_RoboModule_DRV_OpenLoop_Mode(0,8,Rotate_Speed);
		}break;
		
		default:
		{
		}
	}
		
}

void Set_Arm_Joint_Angular_Velocity(u8 Joint,float Angular_Velocity)  //(arc/s)
{
	float rotate_velocity=0;

	if(Joint == 1)
	{
		rotate_velocity =  1.0*(Angular_Velocity*30*SCALE1/PI);
		if((Arm.Joint_1_Limit_Switch==1)&&(rotate_velocity<0))
		{
			rotate_velocity = 0;
		}
		if((Arm.Joint_1_Limit_Switch==2)&&(rotate_velocity>0))
		{
			rotate_velocity = 0;
		}
	}
	else if(Joint == 2)
	{
		rotate_velocity =  1.0*(Angular_Velocity*30*SCALE2/PI);
		if((Arm.Joint_2_Limit_Switch==2)&&(rotate_velocity<0))
		{
			rotate_velocity = 0;
		}
		if((Arm.Joint_2_Limit_Switch==1)&&(rotate_velocity>0))
		{
			rotate_velocity = 0;
		}
	}
	else if(Joint == 3)
	{
		rotate_velocity =  1.0*(Angular_Velocity*30*SCALE3/PI);
		if((Arm.Joint_3_Limit_Switch==1)&&(rotate_velocity<0))
		{
			rotate_velocity = 0;
		}
		if((Arm.Joint_3_Limit_Switch==2)&&(rotate_velocity>0))
		{
			rotate_velocity = 0;
		}
	}
	else if(Joint == 4)
	{
		rotate_velocity =  1.0*(Angular_Velocity*30*SCALE4/PI);
		if((Arm.Joint_4_Limit_Switch==2)&&(rotate_velocity<0))
		{
			rotate_velocity = 0;
		}
		if((Arm.Joint_4_Limit_Switch==1)&&(rotate_velocity>0))
		{
			rotate_velocity = 0;
		}
	}
	else
	{
		rotate_velocity=0;
	}
	
	if(rotate_velocity> 6000) rotate_velocity =  6000;
	if(rotate_velocity<-6000) rotate_velocity = -6000;
	 
		Set_Arm_Joint_Motor_Rotate_Speed(Joint,rotate_velocity);
}


void Set_Arm_Joint_4_Angle(float Target_Angle)
{
	static float e_Angle=0,Last_e_Angle=0;
	static float PID_I_Out=0;
	//float Kp=4.6,Kd=7.0,Ki=0.0,Target_Angular_Velocity=0;
	float Kp=3.6,Kd=5.0,Ki=0.0,Target_Angular_Velocity=0;
	
	Last_e_Angle = e_Angle;
	e_Angle = Target_Angle - Arm.Joint_4_Angle;
	
	if(e_Angle>5) e_Angle=5;
	if(e_Angle<-5) e_Angle=-5;
	
	PID_I_Out += Ki*e_Angle;
	if(PID_I_Out>3) PID_I_Out=3;
	if(PID_I_Out<-3) PID_I_Out=-3;
	
	if(my_fabs(e_Angle)<0.3) 
	{
		e_Angle=0;
		PID_I_Out=0;
	}
	
	//Target_Angular_Velocity = Kp*my_fabs(e_Angle)*e_Angle + Kd * (e_Angle - Last_e_Angle);
	Target_Angular_Velocity = Kp*e_Angle + Kd * (e_Angle - Last_e_Angle) + PID_I_Out;
	
	if(Target_Angular_Velocity>8) Target_Angular_Velocity = 8;
	if(Target_Angular_Velocity<-8) Target_Angular_Velocity = -8;
	
	Target_Angular_Velocity = Target_Angular_Velocity*PI/180;
	
	Set_Arm_Joint_Angular_Velocity(4,Target_Angular_Velocity);
}

void Get_All_Joint_Angle_And_Coordinate(void)
{
//	if(Arm.Joint_1_Limit_Switch==1)
//	{
//		Arm.Joint_1_Reset_Position = Get_Joint_Motor_Pos(1);
//	}
	
	if(Arm.Joint_2_Limit_Switch==1)
	{
		Arm.Joint_2_Reset_Position = Get_Joint_Motor_Pos(2);
	}
	if(Arm.Joint_3_Limit_Switch==1)
	{
		Arm.Joint_3_Reset_Position = Get_Joint_Motor_Pos(3);
	}
	if(Arm.Joint_4_Limit_Switch==2)
	{
		Arm.Joint_4_Reset_Position = Get_Joint_Motor_Pos(4);
	}
	
	Arm.Joint_1_Angle = Get_Joint_Angle(1, Get_Joint_Motor_Pos(1)-Arm.Joint_1_Reset_Position); // degree
	Arm.Joint_2_Angle = Get_Joint_Angle(2, Get_Joint_Motor_Pos(2)-Arm.Joint_2_Reset_Position);
	Arm.Joint_3_Angle = Get_Joint_Angle(3, Get_Joint_Motor_Pos(3)-Arm.Joint_3_Reset_Position);
	Arm.Joint_4_Angle = Get_Joint_Angle(4, Get_Joint_Motor_Pos(4)-Arm.Joint_4_Reset_Position);
	
	////----------------------正解 计算机械臂末端位置 ---------------------////
	
	Theta_1 = Arm.Joint_1_Angle*PI/180;  // rad
	Theta_2 = Arm.Joint_2_Angle*PI/180;
	Theta_3 = Arm.Joint_3_Angle*PI/180;
	Theta_4 = Arm.Joint_4_Angle*PI/180;
	
//	Arm.Joint_4_Coordinate.X = cos(Theta_1)*(L2*sin(Theta_2) + L3*sin(Theta_2+Theta_3) + L4*sin(Theta_2+Theta_3+Theta_4));
//	Arm.Joint_4_Coordinate.Y = L2*cos(Theta_2) + L3*cos(Theta_2+Theta_3) + L4*cos(Theta_2+Theta_3+Theta_4);
	
	Arm.Joint_3_Coordinate.X = L2*sin(Theta_2) + L3*sin(Theta_2+Theta_3);
	Arm.Joint_3_Coordinate.Y = L2*cos(Theta_2) + L3*cos(Theta_2+Theta_3);
//	Arm.Joint_4_Coordinate.Z = sin(Theta_1)*(L2*sin(Theta_2) + L3*sin(Theta_2+Theta_3) + L4*sin(Theta_2+Theta_3+Theta_4));

	Arm.Joint_4_Coordinate.X = L2*sin(Theta_2) + L3*sin(Theta_2+Theta_3) + L4*sin(Theta_2+Theta_3+Theta_4);
	Arm.Joint_4_Coordinate.Y = L2*cos(Theta_2) + L3*cos(Theta_2+Theta_3) + L4*cos(Theta_2+Theta_3+Theta_4);

}


///////////------------------------- 联动 -------------------------//////////

void Arm_Link_Control(void)
{
	static float MAX_Velocity=0.04;
	static u8 once=0;
	static int count=0;
	
	////-------------------------------关节1->节点1-------------------------------/////
		
	//if((Arm_Rocker.Key5==1)&&(my_abs(Arm_Rocker.X)>8)) //  关节1-----大臂左右  负值是向后走
	if(Arm_Rocker.Key5==1)
	{
		if((Arm.Joint_1_Limit_Switch!=1)&&(Arm_Rocker.X<0))
		{
			Set_Arm_Joint_Motor_Rotate_Speed(1, SPEED_SCALE*Arm_Rocker.X);//
		}
		else 
		if((Arm.Joint_1_Limit_Switch!=2)&&(Arm_Rocker.X>0))
		{
			Set_Arm_Joint_Motor_Rotate_Speed(1, SPEED_SCALE*Arm_Rocker.X);//
		}
		else
		{
			Set_Arm_Joint_Motor_Rotate_Speed(1, 0);		
		}	
	}
	else
	{
		Set_Arm_Joint_Motor_Rotate_Speed(1, 0);	
	}	
	////--------------------  get arm joint angle -----------------////
	
	//Get_All_Joint_Angle_And_Coordinate();
	
//	if(once<10)
//	{
//		once++;
//		Arm_Reset_X = L2*sin(Theta_2) + L3*sin(Theta_2+Theta_3) + L4*sin(Theta_2+Theta_3+Theta_4);
//		Arm_Reset_Y = L2*cos(Theta_2) + L3*cos(Theta_2+Theta_3) + L4*cos(Theta_2+Theta_3+Theta_4);
//	}
	
	////------------------------ 位置闭环PID控制 + 反解计算关节角速度 ------------------------////

	if((Arm_Rocker.Key1==1)&&(Arm_Rocker.Key2==1))
	{
		if(Arm_Rocker.Key5==1) //	
		{	
			////--------------------------- 关节2,3->节点2,3 -----------------------------////
			
//			if(my_abs(Arm_Rocker.Y)<10) Arm_Rocker.Y=0;
//			if(my_abs(Arm_Rocker.Z1)<10) Arm_Rocker.Z1=0;
			
			if((Arm.Joint_3_Coordinate.X*Arm.Joint_3_Coordinate.X + Arm.Joint_3_Coordinate.Y*Arm.Joint_3_Coordinate.Y)>0.64)  // safety work space
			{
				if(Arm_Rocker.Y<0)
				{
					Arm_Rocker.Y=0;
				}		
				if(Arm_Rocker.Z1>0)
				{
					Arm_Rocker.Z1=0;
				}		
			}
			
			Arm_Target_X -= 0.0000015*Arm_Rocker.Y; 
			Arm_Target_Y += 0.0000015*Arm_Rocker.Z1;
			
//			if(Arm_Target_X>0.4) Arm_Target_X = 0.4;
//			if(Arm_Target_Y>0.4) Arm_Target_Y = 0.4;
//			if(Arm_Target_X<-0.4) Arm_Target_X = -0.4;
//			if(Arm_Target_Y<-0.4) Arm_Target_Y = -0.4;
			
			Arm_X_Offset_Last = Arm_X_Offset;
			Arm_Y_Offset_Last = Arm_Y_Offset; 
			
			Arm_X_Offset = Arm.Joint_4_Coordinate.X - Arm_Reset_X - Arm_Target_X;
			Arm_Y_Offset = Arm.Joint_4_Coordinate.Y - Arm_Reset_Y - Arm_Target_Y;
			
			Arm_Target_Speed_X = Arm_Kp * Arm_X_Offset + Arm_Kd * (Arm_X_Offset - Arm_X_Offset_Last);
			Arm_Target_Speed_Y = Arm_Kp * Arm_Y_Offset + Arm_Kd * (Arm_Y_Offset - Arm_Y_Offset_Last);
			
			if(Arm_Target_Speed_X>MAX_Velocity) Arm_Target_Speed_X=MAX_Velocity;
			if(Arm_Target_Speed_Y>MAX_Velocity) Arm_Target_Speed_Y=MAX_Velocity;
			if(Arm_Target_Speed_X<-MAX_Velocity) Arm_Target_Speed_X=-MAX_Velocity;
			if(Arm_Target_Speed_Y<-MAX_Velocity) Arm_Target_Speed_Y=-MAX_Velocity;
			
//			if((Arm.Joint_3_Coordinate.X*Arm.Joint_3_Coordinate.X + Arm.Joint_3_Coordinate.Y*Arm.Joint_3_Coordinate.Y)>0.64)  // safety work space
//			{
//				if(Arm_Rocker.Y<0)
//				{
//					Arm_Target_Speed_X = 0;
//					Arm_Target_Speed_Y = 0;
//				}		
//				if(Arm_Rocker.Z1>0)
//				{
//					Arm_Target_Speed_X = 0;
//					Arm_Target_Speed_Y = 0;
//				}		
//			}
			
		  ////-------------------------- joint 2 , 3 control --------------------------------////
			 
			A1=L3*cos(Theta_2+Theta_3)+ L2*cos(Theta_2);
			B1=L3*cos(Theta_2+Theta_3);
			A2=-L3*sin(Theta_2+Theta_3)- L2*sin(Theta_3);
			B2=-L3*sin(Theta_2+Theta_3);

			d_Theta_2=(B2*Arm_Target_Speed_X-B1*Arm_Target_Speed_Y)/(A1*B2-A2*B1);//
			d_Theta_3=(A2*Arm_Target_Speed_X-A1*Arm_Target_Speed_Y)/(B1*A2-A1*B2);//(rad/s)		
			
//			if(my_fabs(d_Theta_2)>0.10472) //    6/180*3.14159=0.10472 (rad/s)
//			{
//				d_Theta_3 = d_Theta_3*0.10472/d_Theta_2;
//				if(d_Theta_2>0.10472) d_Theta_2=0.10472;
//				if(d_Theta_2<-0.10472) d_Theta_2=-0.10472;
//			}

//			if(Arm.Joint_2_Angle>70 || Arm.Joint_2_Angle<-70) 
//			{
//				d_Theta_2=0;
//				d_Theta_3=0;
//			}
//			if(Arm.Joint_3_Angle>160 || Arm.Joint_3_Angle<20) 
//			{
//				d_Theta_2=0;
//				d_Theta_3=0;
//			}
			
			Set_Arm_Joint_Angular_Velocity(2, d_Theta_2);
			Set_Arm_Joint_Angular_Velocity(3, d_Theta_3);
			
			////-------------------- joint 4 control ---------------------//////
				
//			Target_Joint_4_Angle = Arm.Terminal_Hold_Angle + 90 - Arm.Joint_2_Angle - Arm.Joint_3_Angle;
//			Set_Arm_Joint_4_Angle(Target_Joint_4_Angle);
		}			
		else
		{
			Set_Arm_Joint_Angular_Velocity(2,0);
			Set_Arm_Joint_Angular_Velocity(3,0);
			//Set_Arm_Joint_Angular_Velocity(4,0);
			
			Arm_Reset_X = Arm.Joint_4_Coordinate.X;
			Arm_Reset_Y = Arm.Joint_4_Coordinate.Y;
			
			Arm_Target_X = 0;
			Arm_Target_Y = 0;
		}		
		
		////------------------------------ joint 4 control -------------------------------//////
		
		if(my_abs(Arm_Rocker.Z2)<8)
		{
			Arm_Rocker.Z2=0;
		}
		Arm.Terminal_Hold_Angle += 0.0002*Arm_Rocker.Z2;
		Target_Joint_4_Angle = Arm.Terminal_Hold_Angle + 90 - Arm.Joint_2_Angle - Arm.Joint_3_Angle;
		Set_Arm_Joint_4_Angle(Target_Joint_4_Angle);
	}
	else
	{
		Set_Arm_Joint_Angular_Velocity(2,0);
		Set_Arm_Joint_Angular_Velocity(3,0);
		Set_Arm_Joint_Angular_Velocity(4,0);
		//Arm.Terminal_Hold_Angle = Arm.Joint_2_Angle + Arm.Joint_3_Angle + Arm.Joint_4_Angle - 90;		
	}
	
	////------------------------------ joint 5,6 control -------------------------------//////
	if(Arm_Rocker.Key3==1) 
	{
		if(Arm_Rocker.Key5==1)
		{
			Set_Arm_Joint_Motor_Rotate_Speed(6, 20*Arm_Rocker.Speed_Adjust);  
			Set_Arm_Joint_Motor_Rotate_Speed(5, 0); 
		}
		else
		{
			Set_Arm_Joint_Motor_Rotate_Speed(6, 0);  
			Set_Arm_Joint_Motor_Rotate_Speed(5, 20*Arm_Rocker.Speed_Adjust);
		}
		
	}
	else
	if(Arm_Rocker.Key4==1)
	{
		if(Arm_Rocker.Key5==1)
		{
			Set_Arm_Joint_Motor_Rotate_Speed(6, -20*Arm_Rocker.Speed_Adjust); 
			Set_Arm_Joint_Motor_Rotate_Speed(5, 0); 
		}
		else
		{
			Set_Arm_Joint_Motor_Rotate_Speed(6, 0); 
			Set_Arm_Joint_Motor_Rotate_Speed(5, -20*Arm_Rocker.Speed_Adjust); 
		}
	}
	else
	{
		Set_Arm_Joint_Motor_Rotate_Speed(5, 0);	
		Set_Arm_Joint_Motor_Rotate_Speed(6, 0);	
	}	

}

void Arm_Line_Control(u8 move_enable,double speed_x,double speed_y)
{
  float MAX_Velocity=0.04;
	
	if(move_enable == 1) //	
	{	
		////--------------------------- 关节2,3->节点2,3 -----------------------------////
		
		if((Arm.Joint_3_Coordinate.X*Arm.Joint_3_Coordinate.X + Arm.Joint_3_Coordinate.Y*Arm.Joint_3_Coordinate.Y)>0.64)  // safety work space
		{
			if(speed_x>0)
			{
				speed_x=0;
			}		
			if(speed_y>0)
			{
				speed_y=0;
			}		
		}
		
		Arm_Target_X += 0.0000015*speed_x;  // speed_x = 100 >> 3cm/s
		Arm_Target_Y += 0.0000015*speed_y;
		
		Arm_X_Offset_Last = Arm_X_Offset;
		Arm_Y_Offset_Last = Arm_Y_Offset; 
		
		Arm_X_Offset = Arm.Joint_4_Coordinate.X - Arm_Reset_X - Arm_Target_X;
		Arm_Y_Offset = Arm.Joint_4_Coordinate.Y - Arm_Reset_Y - Arm_Target_Y;
		
		Arm_Target_Speed_X = Arm_Kp * Arm_X_Offset + Arm_Kd * (Arm_X_Offset - Arm_X_Offset_Last);
		Arm_Target_Speed_Y = Arm_Kp * Arm_Y_Offset + Arm_Kd * (Arm_Y_Offset - Arm_Y_Offset_Last);
		
		if(Arm_Target_Speed_X>MAX_Velocity) Arm_Target_Speed_X=MAX_Velocity;
		if(Arm_Target_Speed_Y>MAX_Velocity) Arm_Target_Speed_Y=MAX_Velocity;
		if(Arm_Target_Speed_X<-MAX_Velocity) Arm_Target_Speed_X=-MAX_Velocity;
		if(Arm_Target_Speed_Y<-MAX_Velocity) Arm_Target_Speed_Y=-MAX_Velocity;
		
//			if((Arm.Joint_3_Coordinate.X*Arm.Joint_3_Coordinate.X + Arm.Joint_3_Coordinate.Y*Arm.Joint_3_Coordinate.Y)>0.64)  // safety work space
//			{
//				if(Arm_Rocker.Y<0)
//				{
//					Arm_Target_Speed_X = 0;
//					Arm_Target_Speed_Y = 0;
//				}		
//				if(Arm_Rocker.Z1>0)
//				{
//					Arm_Target_Speed_X = 0;
//					Arm_Target_Speed_Y = 0;
//				}		
//			}
		
		////-------------------------- joint 2 , 3 control --------------------------------////
		 
		A1=L3*cos(Theta_2+Theta_3)+ L2*cos(Theta_2);
		B1=L3*cos(Theta_2+Theta_3);
		A2=-L3*sin(Theta_2+Theta_3)- L2*sin(Theta_3);
		B2=-L3*sin(Theta_2+Theta_3);

		d_Theta_2=(B2*Arm_Target_Speed_X-B1*Arm_Target_Speed_Y)/(A1*B2-A2*B1);//
		d_Theta_3=(A2*Arm_Target_Speed_X-A1*Arm_Target_Speed_Y)/(B1*A2-A1*B2);//(rad/s)		
		
		Set_Arm_Joint_Angular_Velocity(2, d_Theta_2);
		Set_Arm_Joint_Angular_Velocity(3, d_Theta_3);
		
		////------------------------------ joint 4 control -------------------------------//////
	
		if(my_abs(Arm_Rocker.Z2)<8)
		{
			Arm_Rocker.Z2=0;
		}
		Arm.Terminal_Hold_Angle += 0.0002*Arm_Rocker.Z2;
		Target_Joint_4_Angle = Arm.Terminal_Hold_Angle + 90 - Arm.Joint_2_Angle - Arm.Joint_3_Angle;
		Set_Arm_Joint_4_Angle(Target_Joint_4_Angle);
	}			
	else
	{
		Set_Arm_Joint_Angular_Velocity(2,0);
		Set_Arm_Joint_Angular_Velocity(3,0);
		Set_Arm_Joint_Angular_Velocity(4,0);
		
		Arm_Reset_X = Arm.Joint_4_Coordinate.X;
		Arm_Reset_Y = Arm.Joint_4_Coordinate.Y;
		
		Arm_Target_X = 0;
		Arm_Target_Y = 0;
	}		
	
}

//////--------------单关节运动------------////////////////
//////--------------单关节运动 + 末端姿态保持------------////////////////

void Arm_Joint_Control(void)
{
	////-------------------------------关节1->节点1-------------------------------/////
		
	//if((Arm_Rocker.Key5==1)&&(my_abs(Arm_Rocker.X)>10)) //  关节1-----大臂左右  负值是向后走
	if(Arm_Rocker.Key5==1)
	{
		if((Arm.Joint_1_Limit_Switch!=1)&&(Arm_Rocker.X<0))
		{
			Set_Arm_Joint_Motor_Rotate_Speed(1, SPEED_SCALE*Arm_Rocker.X);//
		}
		else 
		if((Arm.Joint_1_Limit_Switch!=2)&&(Arm_Rocker.X>0))
		{
			Set_Arm_Joint_Motor_Rotate_Speed(1, SPEED_SCALE*Arm_Rocker.X);//
		}
		else
		{
			Set_Arm_Joint_Motor_Rotate_Speed(1, 0);		
		}	
	}
	else
	{
		Set_Arm_Joint_Motor_Rotate_Speed(1, 0);	
	}	
		
	////------------------------------- 关节2,3,4->节点2,3,4------------------------------////
	
	if((Arm_Rocker.Key1 + Arm_Rocker.Key2)==1)
	{	
		////-------------------- 关节2->节点2----------------------////
		
		//if((Arm_Rocker.Key5==1)&&(my_abs(Arm_Rocker.Y)>10)) //  关节1-----大臂左右  负值是向后走
		if(Arm_Rocker.Key5==1)
		{
			
			if((Arm.Joint_2_Limit_Switch!=2)&&(Arm_Rocker.Y<0))
			{
				Set_Arm_Joint_Motor_Rotate_Speed(2, SPEED_SCALE*Arm_Rocker.Y);//
			}
			else 
			if((Arm.Joint_2_Limit_Switch!=1)&&(Arm_Rocker.Y>0))
			{
				Set_Arm_Joint_Motor_Rotate_Speed(2, SPEED_SCALE*Arm_Rocker.Y);//
			}
			else
			{
				Set_Arm_Joint_Motor_Rotate_Speed(2, 0);		
			}	
		}
		else
		{
			Set_Arm_Joint_Motor_Rotate_Speed(2, 0);	
		}	
			
		////-------------------- 关节3->节点3----------------------////		
		
		//if((Arm_Rocker.Key5==1)&&(my_abs(Arm_Rocker.Z1)>10)) //  关节1-----大臂左右  负值是向后走
		if(Arm_Rocker.Key5==1)
		{
			
			if((Arm.Joint_3_Limit_Switch!=1)&&(Arm_Rocker.Z1<0))
			{
				Set_Arm_Joint_Motor_Rotate_Speed(3, SPEED_SCALE/2*Arm_Rocker.Z1);//
			}
			else 
			if((Arm.Joint_3_Limit_Switch!=2)&&(Arm_Rocker.Z1>0))
			{
				Set_Arm_Joint_Motor_Rotate_Speed(3, SPEED_SCALE/2*Arm_Rocker.Z1);//
			}
			else
			{
				Set_Arm_Joint_Motor_Rotate_Speed(3, 0);		
			}	
		}
		else
		{
			Set_Arm_Joint_Motor_Rotate_Speed(3, 0);	
		}	
		
		////---------------------- 关节4->节点4-------------------------//	
		
		if(Arm_Rocker.Key2==1)
		{
			if(my_abs(Arm_Rocker.Z2)<5)
			{
				Arm_Rocker.Z2=0;
			}
			Arm.Terminal_Hold_Angle += 0.0002*Arm_Rocker.Z2;
			Target_Joint_4_Angle = Arm.Terminal_Hold_Angle + 90 - Arm.Joint_2_Angle - Arm.Joint_3_Angle;
			Set_Arm_Joint_4_Angle(Target_Joint_4_Angle);
		}
		else if(Arm_Rocker.Key1==1)
		{
			//if((Arm_Rocker.Key5==1)&&(my_abs(Arm_Rocker.Z2)>10)) //  关节1-----大臂左右  负值是向后走
			if(Arm_Rocker.Key5==1)
			{			
				if((Arm.Joint_4_Limit_Switch!=2)&&(Arm_Rocker.Z2<0))
				{
					Set_Arm_Joint_Motor_Rotate_Speed(4, SPEED_SCALE*Arm_Rocker.Z2);//
				}
				else 
				if((Arm.Joint_4_Limit_Switch!=1)&&(Arm_Rocker.Z2>0))
				{
					Set_Arm_Joint_Motor_Rotate_Speed(4, SPEED_SCALE*Arm_Rocker.Z2);
				}
				else
				{
					Set_Arm_Joint_Motor_Rotate_Speed(4, 0);		
				}	
			}
			else
			{
				Set_Arm_Joint_Motor_Rotate_Speed(4, 0);		
			}	
	  }
		else
		{
			Set_Arm_Joint_Motor_Rotate_Speed(4, 0);	
		}	
		
	}
	else
	{
		Set_Arm_Joint_Motor_Rotate_Speed(2, 0);	
		Set_Arm_Joint_Motor_Rotate_Speed(3, 0);	
		Set_Arm_Joint_Motor_Rotate_Speed(4, 0);	
	}	
	
	////----------------------------- 关节5,6->节点5,6--------------------------------////
	
	if(Arm_Rocker.Key3==1) 
	{
		if(Arm_Rocker.Key5==1)
		{
			Set_Arm_Joint_Motor_Rotate_Speed(6, 20*Arm_Rocker.Speed_Adjust);  
			Set_Arm_Joint_Motor_Rotate_Speed(5, 0); 
		}
		else
		{
			Set_Arm_Joint_Motor_Rotate_Speed(6, 0);  
			Set_Arm_Joint_Motor_Rotate_Speed(5, 20*Arm_Rocker.Speed_Adjust);
		}
		
	}
	else
	if(Arm_Rocker.Key4==1)
	{
		if(Arm_Rocker.Key5==1)
		{
			Set_Arm_Joint_Motor_Rotate_Speed(6, -20*Arm_Rocker.Speed_Adjust); 
			Set_Arm_Joint_Motor_Rotate_Speed(5, 0); 
		}
		else
		{
			Set_Arm_Joint_Motor_Rotate_Speed(6, 0); 
			Set_Arm_Joint_Motor_Rotate_Speed(5, -20*Arm_Rocker.Speed_Adjust); 
		}
	}
	else
	{
		Set_Arm_Joint_Motor_Rotate_Speed(5, 0);	
		Set_Arm_Joint_Motor_Rotate_Speed(6, 0);	
	}	
	
}	

//////-------------- 一键伸展，一键复位 -----------////////////////

void Arm_Auto_Stretch_And_Reset(void)
{
	double temp_speed=0,speed_limit=6;
	if((Arm_Rocker.Key1==0)&&(Arm_Rocker.Key2==0))
	{
		if((Arm_Rocker.Key5==1)&&(Arm_Rocker.Y>100)) //-------- Reset --------//
		{
			
			if(my_fabs(Arm.Joint_1_Angle)>0.1)  // to reset position
			{
				temp_speed = -5.0*Arm.Joint_1_Angle;
				if(temp_speed>12) temp_speed=12;
				if(temp_speed<-12) temp_speed=-12;				
				temp_speed = temp_speed*PI/180;
				
				Set_Arm_Joint_Angular_Velocity(1, temp_speed);
			}
			else
			{
				Set_Arm_Joint_Angular_Velocity(1, 0);
			}
			
			if(Arm.Joint_2_Limit_Switch!=1)
			{
				Set_Arm_Joint_Angular_Velocity(2, 5.0*PI/180);
			}
			else
			{
				Set_Arm_Joint_Angular_Velocity(2, 0);
			}
		
	
			if(Arm.Joint_3_Limit_Switch!=1)
			{
				Set_Arm_Joint_Angular_Velocity(3, -5.0*PI/180);
			}
			else
			{
				Set_Arm_Joint_Angular_Velocity(3, 0);
			}
		
			if(Arm.Joint_4_Limit_Switch!=2)
			{
				Set_Arm_Joint_Angular_Velocity(4, -5.0*PI/180);
			}
			else
			{
				Set_Arm_Joint_Angular_Velocity(4, 0);
			}
			
			
//			if(my_fabs(Arm.Joint_1_Angle)>0.1)  // to reset position
//			{
//				temp_speed = -5.0*Arm.Joint_1_Angle;
//				if(temp_speed>12) temp_speed=12;
//				if(temp_speed<-12) temp_speed=-12;				
//				temp_speed = temp_speed*PI/180;
//				
//				Set_Arm_Joint_Angular_Velocity(1, temp_speed);
//			}
//			else
//			{
//				Set_Arm_Joint_Angular_Velocity(1, 0);
//			}
//			
//			if(my_fabs(Arm.Joint_2_Angle-(0+Joint_2_offset_Angle))>0.1)  // to reset position
//			{
//				temp_speed = 5.0*(Arm.Joint_2_Angle - (Joint_2_offset_Angle));
//				if(temp_speed>speed_limit) temp_speed=speed_limit;
//				if(temp_speed<-speed_limit) temp_speed=-speed_limit;
//				temp_speed = temp_speed*PI/180;
//				
//				Set_Arm_Joint_Angular_Velocity(2, temp_speed);
//			}
//			else
//			{
//				Set_Arm_Joint_Angular_Velocity(2, 0);
//			}
//			
//			if(my_fabs(Arm.Joint_3_Angle-(90.0-Joint_3_offset_Angle))>0.1)  // to reset position
//			{
//				temp_speed = 5.0*(Arm.Joint_3_Angle-(90.0-Joint_3_offset_Angle));
//				if(temp_speed>speed_limit) temp_speed=speed_limit;
//				if(temp_speed<-speed_limit) temp_speed=-speed_limit;
//				temp_speed = temp_speed*PI/180;
//				
//				Set_Arm_Joint_Angular_Velocity(3, temp_speed);
//			}
//			else
//			{
//				Set_Arm_Joint_Angular_Velocity(3, 0);
//			}
//			
//			if(my_fabs(Arm.Joint_4_Angle-(0+Joint_4_offset_Angle))>0.1)  // to reset position
//			{
//				temp_speed = -5.0*(Arm.Joint_4_Angle-(Joint_4_offset_Angle));
//				if(temp_speed>speed_limit) temp_speed=speed_limit;
//				if(temp_speed<-speed_limit) temp_speed=-speed_limit;
//				temp_speed = temp_speed*PI/180;
//				
//				Set_Arm_Joint_Angular_Velocity(4, temp_speed);
//			}
//			else
//			{
//				Set_Arm_Joint_Angular_Velocity(4, 0);
//			}
		}
		else if((Arm_Rocker.Key5==1)&&(Arm_Rocker.Y<-100))  //-------- Stretch --------//
		{
			if(my_fabs(Arm.Joint_1_Angle-(Grasp_Stretch_Point[1]))>0.1)  // to reset position
			{
				temp_speed = -5.0*(Arm.Joint_1_Angle-(Grasp_Stretch_Point[1]));
				if(temp_speed>12) temp_speed=12;
				if(temp_speed<-12) temp_speed=-12;				
				temp_speed = temp_speed*PI/180;
				
				Set_Arm_Joint_Angular_Velocity(1, temp_speed);
			}
			else
			{
				Set_Arm_Joint_Angular_Velocity(1, 0);
			}
			
			if(my_fabs(Arm.Joint_2_Angle-(Grasp_Stretch_Point[2]))>0.1)  // to Stretch position
			{
				temp_speed = 5.0*(Arm.Joint_2_Angle-(Grasp_Stretch_Point[2]));
				if(temp_speed>speed_limit) temp_speed=speed_limit;
				if(temp_speed<-speed_limit) temp_speed=-speed_limit;
				temp_speed = temp_speed*PI/180;
				
				Set_Arm_Joint_Angular_Velocity(2, temp_speed);
			}
			else
			{
				Set_Arm_Joint_Angular_Velocity(2, 0);
			}
			
			if(my_fabs(Arm.Joint_3_Angle-Grasp_Stretch_Point[3])>0.1)  // to Stretch position
			{
				temp_speed = 5.0*(Arm.Joint_3_Angle-Grasp_Stretch_Point[3]);
				if(temp_speed>speed_limit) temp_speed=speed_limit;
				if(temp_speed<-speed_limit) temp_speed=-speed_limit;
				temp_speed = temp_speed*PI/180;
				
				Set_Arm_Joint_Angular_Velocity(3, temp_speed);
			}
			else
			{
				Set_Arm_Joint_Angular_Velocity(3, 0);
			}
			
			if(my_fabs(Arm.Joint_4_Angle-Grasp_Stretch_Point[4])>0.1)  // to Stretch position
			{
				temp_speed = -5.0*(Arm.Joint_4_Angle-Grasp_Stretch_Point[4]);
				if(temp_speed>speed_limit) temp_speed=speed_limit;
				if(temp_speed<-speed_limit) temp_speed=-speed_limit;
				temp_speed = temp_speed*PI/180;
				
				Set_Arm_Joint_Angular_Velocity(4, temp_speed);
			}
			else
			{
				Set_Arm_Joint_Angular_Velocity(4, 0);
			}
					
//			Target_Joint_4_Angle = 5.16 + 90 - Arm.Joint_2_Angle - Arm.Joint_3_Angle;  // 7.16 Arm.Terminal_Hold_Angle = 0
//			Set_Arm_Joint_4_Angle(Target_Joint_4_Angle);
		}
		else
		{
			Set_Arm_Joint_Angular_Velocity(1, 0); // 8/180*3.14159=0.1396 (rad/s)	
			Set_Arm_Joint_Angular_Velocity(2, 0); // 8/180*3.14159=0.1396 (rad/s)				
			Set_Arm_Joint_Angular_Velocity(3, 0); // 6/180*3.14159=0.1047 (rad/s)		
			Set_Arm_Joint_Angular_Velocity(4, 0); // 8/180*3.14159=0.1396 (rad/s)
			Set_Arm_Joint_Motor_Rotate_Speed(5,0);
			Set_Arm_Joint_Motor_Rotate_Speed(6,0);
		}
	}
}

int Arm_To_Preset_Point(float *Point)
{
	int Process_Flag=0;
	double K_P=4.0;
	double temp_speed=0,speed_limit_1=12,speed_limit_2=6,speed_limit_3=6,speed_limit_4=6;
	double Joint_1_Error=0,Joint_2_Error=0,Joint_3_Error=0,Joint_4_Error=0;
	
	Joint_1_Error = Arm.Joint_1_Angle - *(Point+1);
	Joint_2_Error = Arm.Joint_2_Angle - *(Point+2);
	Joint_3_Error = Arm.Joint_3_Angle - *(Point+3);
	Joint_4_Error = Arm.Joint_4_Angle - *(Point+4);
	
	//speed_limit = speed_limit / *(Point+0);
	
	//if(Arm_Rocker.Key5==1) //-------- Reset --------//
	{
		if(my_fabs(Joint_1_Error)>0.1)  // to Reset position
		{
			temp_speed = -K_P*Joint_1_Error;
			//if(my_fabs(Joint_1_Error)<10) speed_limit_1 = speed_limit_1*(10-my_fabs(Joint_1_Error))/10;
			if(temp_speed>speed_limit_1) temp_speed=speed_limit_1;
			if(temp_speed<-speed_limit_1) temp_speed=-speed_limit_1;
			temp_speed = temp_speed*PI/180;
			
			Set_Arm_Joint_Angular_Velocity(1, temp_speed);
		}
		else
		{
			Set_Arm_Joint_Angular_Velocity(1, 0);
			Process_Flag = Process_Flag | 0x01;
		}
		
		if(my_fabs(Joint_2_Error)>0.1)  // to Stretch position
		{
			temp_speed = K_P*Joint_2_Error;
			//if(my_fabs(Joint_2_Error)<10) speed_limit_2 = speed_limit_2*(10-my_fabs(Joint_2_Error))/10;
			if(temp_speed>speed_limit_2) temp_speed=speed_limit_2;
			if(temp_speed<-speed_limit_2) temp_speed=-speed_limit_2;
			temp_speed = temp_speed*PI/180;
			
			Set_Arm_Joint_Angular_Velocity(2, temp_speed);
		}
		else
		{
			Set_Arm_Joint_Angular_Velocity(2, 0);
			Process_Flag = Process_Flag | 0x02;
		}
		
		if(my_fabs(Joint_3_Error)>0.1)  // to Stretch position
		{
			temp_speed = K_P*Joint_3_Error;
			//if(my_fabs(Joint_3_Error)<10) speed_limit_3 = speed_limit_3*(10-my_fabs(Joint_3_Error))/10;
			if(temp_speed>speed_limit_3) temp_speed=speed_limit_3;
			if(temp_speed<-speed_limit_3) temp_speed=-speed_limit_3;
			temp_speed = temp_speed*PI/180;
			
			Set_Arm_Joint_Angular_Velocity(3, temp_speed);
		}
		else
		{
			Set_Arm_Joint_Angular_Velocity(3, 0);
			Process_Flag = Process_Flag | 0x04;
		}
		
		if(my_fabs(Joint_4_Error)>0.1)  // to Stretch position
		{
			temp_speed = -K_P*Joint_4_Error;
			//if(my_fabs(Joint_4_Error)<10) speed_limit_4 = speed_limit_4*(10-my_fabs(Joint_4_Error))/10;
			if(temp_speed>speed_limit_4) temp_speed=speed_limit_4;
			if(temp_speed<-speed_limit_4) temp_speed=-speed_limit_4;
			temp_speed = temp_speed*PI/180;
			
			Set_Arm_Joint_Angular_Velocity(4, temp_speed);
		}
		else
		{
			Set_Arm_Joint_Angular_Velocity(4, 0);
			Process_Flag = Process_Flag | 0x08;
		}
		
//		if(*(Point+5)>0.9)
//		{
//			if(grasp_time<1000)
//			{
//				Set_Arm_Joint_Motor_Rotate_Speed(6, 2000); 
//				grasp_time++;
//			}
//			else
//			{
//				//delay_ms(5);
//				Set_Arm_Joint_Motor_Rotate_Speed(6, 0); 
//				//grasp_time=0;
//				//Process_Flag = Process_Flag | 0x10;
//			}
//		}
		
//		if(Process_Flag==0x0f && *(Point+5)>0.9)
//		{
//			Set_Arm_Joint_Motor_Rotate_Speed(6, -1500); 
//			grasp_time++;
//			if(grasp_time>1500)
//			{
//				delay_ms(2);
//				Set_Arm_Joint_Motor_Rotate_Speed(6, 0); 
//				grasp_time=0;
//				Process_Flag = Process_Flag | 0x10;
//			}
//		}
		
	}
//	else
//	{
//		Process_Flag=0;
//		Set_Arm_Joint_Angular_Velocity(1, 0); // 8/180*3.14159=0.1396 (rad/s)	
//		Set_Arm_Joint_Angular_Velocity(2, 0); // 8/180*3.14159=0.1396 (rad/s)				
//		Set_Arm_Joint_Angular_Velocity(3, 0); // 6/180*3.14159=0.1047 (rad/s)		
//		Set_Arm_Joint_Angular_Velocity(4, 0); // 8/180*3.14159=0.1396 (rad/s)
//		Set_Arm_Joint_Motor_Rotate_Speed(5,0);
//		Set_Arm_Joint_Motor_Rotate_Speed(6,0);
//	}
	
	return Process_Flag;
	
}

int Arm_Auto_Grasp_Process(float *Point1,float *Point2,float *Point3,float *Point4,float *Point5,u8 grasp)
{
	//static int Process_Flag_1=0,Process_Flag_2=0,Process_Flag_3=0,Process_Flag_4=0,Process_Flag_5=0;
	//static int grasp_time=0;
	//static int open_grasp=0;
	
	open_grasp++;
	
	if(Process_Flag_1 != 0x0f)
	{
		Process_Flag_1 = Arm_To_Preset_Point(Point1);
		if(open_grasp<1200)
		{
			Set_Arm_Joint_Motor_Rotate_Speed(6, 2000); 
		}
		else
		{
			Set_Arm_Joint_Motor_Rotate_Speed(6, 0); 
		}
	}
	else if(Process_Flag_2 != 0x0f)
	{
		Process_Flag_2 = Arm_To_Preset_Point(Point2);
		Set_Arm_Joint_Motor_Rotate_Speed(6, 0); 
		grasp_time=0;
		open_grasp=0;
	}
	else if(Process_Flag_3 != 0x01)
	{
		if(grasp==1)
		{
			Set_Arm_Joint_Motor_Rotate_Speed(6, -1500);  // close
		}
		else
		{
			Set_Arm_Joint_Motor_Rotate_Speed(6, 1500); 
		}
		
		grasp_time++;
		if(grasp_time>1600)
		{
			delay_ms(5);
			Set_Arm_Joint_Motor_Rotate_Speed(6, 0); 
			grasp_time=0;
			Process_Flag_3 = 0x01;
		}
	}
	else if(Process_Flag_4 != 0x0f)
	{
		Process_Flag_4 = Arm_To_Preset_Point(Point3);
	}
	else if(Process_Flag_5 != 0x0f)
	{
		Process_Flag_5 = Arm_To_Preset_Point(Point4);
	}
	else if(Process_Flag_6 != 0x0f)
	{
		Process_Flag_6 = Arm_To_Preset_Point(Point5);
	}
	else
	{
		Process_Flag_1=0;
		Process_Flag_2=0;
		Process_Flag_3=0;
		Process_Flag_4=0;
		Process_Flag_5=0;
		Process_Flag_6=0;
		grasp_time=0;
		open_grasp=0;
		Arm_Auto_Grasp_Flag=0;
		return 1;
	}
	
	return 0;
}

int Arm_Auto_Grasp_return(float *Point1,float *Point2,float *Point3,float *Point4,float *Point5,u8 grasp)
{
	//static int Process_Flag_1=0,Process_Flag_2=0,Process_Flag_3=0,Process_Flag_4=0,Process_Flag_5=0;
	//static int grasp_time=0;
	
	if(Process_Flag_1 != 0x0f)
	{
		Process_Flag_1 = Arm_To_Preset_Point(Point1);
	}
	else if(Process_Flag_2 != 0x0f)
	{
		Process_Flag_2 = Arm_To_Preset_Point(Point2);
	}
	else if(Process_Flag_4 != 0x0f)
	{
		Process_Flag_4 = Arm_To_Preset_Point(Point3);
	}
	else if(Process_Flag_3 != 0x01)
	{
		if(grasp==1)
		{
			Set_Arm_Joint_Motor_Rotate_Speed(6, -1500); 
		}
		else
		{
			Set_Arm_Joint_Motor_Rotate_Speed(6, 3000); 
		}
		
		grasp_time++;
		if(grasp_time>800)
		{
			delay_ms(5);
			Set_Arm_Joint_Motor_Rotate_Speed(6, 0); 
			grasp_time=0;
			Process_Flag_3 = 0x01;
		}
	}
	else if(Process_Flag_5 != 0x0f)
	{
		Process_Flag_5 = Arm_To_Preset_Point(Point4);
	}
	else if(Process_Flag_6 != 0x0f)
	{
		Process_Flag_6 = Arm_To_Preset_Point(Point5);
	}
	else
	{
		Process_Flag_1=0;
		Process_Flag_2=0;
		Process_Flag_3=0;
		Process_Flag_4=0;
		Process_Flag_5=0;
		Process_Flag_6=0;
		grasp_time=0;
		open_grasp=0;
		Arm_Auto_Grasp_Flag=0;
		return 1;
	}
	
	return 0;
}

void Arm_Stop(void)
{
	Set_Arm_Joint_Motor_Rotate_Speed(1,0);
	Set_Arm_Joint_Motor_Rotate_Speed(2,0);
	Set_Arm_Joint_Motor_Rotate_Speed(3,0);
	Set_Arm_Joint_Motor_Rotate_Speed(4,0);
	Set_Arm_Joint_Motor_Rotate_Speed(5,0);
	Set_Arm_Joint_Motor_Rotate_Speed(6,0);
}

//////--------------------- 云台控制 ---------------------////////////////

void Camera_Platform_Control(void)
{
	Set_Arm_Joint_Motor_Rotate_Speed(7, Yaw_Speed);	
	Set_Arm_Joint_Motor_Rotate_Speed(8, Ptich_Speed);	
}

//////--------------------- 手臂控制 ---------------------////////////////

void Arm_Control(void)
{
	static int count;
	
	Get_All_Joint_Angle_And_Coordinate();
	
	if((Arm_Rocker.Key1==0)&&(Arm_Rocker.Key2==0))
	{
		Arm_Control_Mode = 1;
	}
	else if((Arm_Rocker.Key1==1)&&(Arm_Rocker.Key2==0))
	{
		Arm_Control_Mode = 2;
	}
	else if((Arm_Rocker.Key1==0)&&(Arm_Rocker.Key2==1))
	{
		Arm_Control_Mode = 3;
	}
	else if((Arm_Rocker.Key1==1)&&(Arm_Rocker.Key2==1))
	{
		Arm_Control_Mode = 4;
	}
	else
	{
		Arm_Control_Mode = 0;
	}
	
	if(Arm_Control_Mode!=1)
	{
		Process_Flag_1=0;
		Process_Flag_2=0;
		Process_Flag_3=0;
		Process_Flag_4=0;
		Process_Flag_5=0;
		Process_Flag_6=0;
		grasp_time=0;
		open_grasp=0;
		Arm_Auto_Grasp_Flag=0;
	}
	
	if(Arm_Control_Mode==1)
	{
		Arm.Terminal_Hold_Angle = Arm.Joint_2_Angle + Arm.Joint_3_Angle + Arm.Joint_4_Angle - 90;
			
		if((Arm_Rocker.Key3==1)&&(Arm_Rocker.Key5==1))
		{
			Arm_Auto_Grasp_Flag = 1;
		}
		
		if((Arm_Rocker.Key4==1)&&(Arm_Rocker.Key5==1))
		{
			Arm_Auto_Grasp_Flag = 2;
		}
		
		if(Arm_Auto_Grasp_Flag==1)
		{
			Arm_Auto_Grasp_Process(Preset_Point_1_up,Preset_Point_1_down,Preset_Point_1_up,Preset_Point_1_grasp_up,Grasp_Stretch_Point,1);
		}
		else if(Arm_Auto_Grasp_Flag==2)
		{
			Arm_Auto_Grasp_return(Preset_Point_1_grasp_up,Preset_Point_1_up,Preset_Point_1_up,Preset_Point_1_grasp_up,Grasp_Stretch_Point,2);
		}
		else
		{
			Arm_Auto_Stretch_And_Reset();
		}
		
		if(Robot_Arm_Reset_Flag==1)
		{
			
		}
		
//		if(Arm_Rocker.Key3==1)
//		{
//			Arm_To_Preset_Point(Preset_Point_1_up);
//		}
//		else 
//		if(Arm_Rocker.Key4==1)
//		{
//			Arm_To_Preset_Point(Preset_Point_1_down);
//		}
		
	}
	else if(Arm_Control_Mode==2)
	{
		Arm.Terminal_Hold_Angle = Arm.Joint_2_Angle + Arm.Joint_3_Angle + Arm.Joint_4_Angle - 90;		
		Arm_Joint_Control();	
	}
	else if(Arm_Control_Mode==3)
	{
		Arm_Joint_Control();
	}
	else if(Arm_Control_Mode==4)
	{
		Arm_Link_Control();
	}
	else
	{
		Arm_Stop();
	}
	
	if(count++>30)
	{
		count=0;
//		printf("-Arm_Joint_Angle 1 : %3.1f , 2 : %3.1f , 3 : %3.1f , 4 : %3.1f ",Arm.Joint_1_Angle,Arm.Joint_2_Angle,Arm.Joint_3_Angle,Arm.Joint_4_Angle);
////		printf("-Target_Joint_4_Angle : %3.1f ",Target_Joint_4_Angle); 
////		printf("-Hold_Angle : %3.1f \n",Arm.Terminal_Hold_Angle); 
//		
//		printf("-Joint_3_X  : %3.4f , Y : %3.4f ",Arm.Joint_3_Coordinate.X ,Arm.Joint_3_Coordinate.Y);
//		printf("-Joint_4_X  : %3.4f , Y : %3.4f \n",Arm.Joint_4_Coordinate.X ,Arm.Joint_4_Coordinate.Y);
//		
//		//printf("-Arm_Target_X : %3.4f , Y : %3.4f \n",Arm_Target_X,Arm_Target_Y); // Arm.Terminal_Hold_Angle
//		//printf("-Target_Speed_X : %3.4f , Y : %3.4f \n",Arm_Target_Speed_X,Arm_Target_Speed_Y);
	}
	
		
}







