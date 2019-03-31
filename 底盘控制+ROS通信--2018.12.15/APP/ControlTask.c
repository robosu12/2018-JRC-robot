#include "main.h"

//-------------------suyun define-----------------------//

#define target_weight 0.6
#define speed_ref_limit 4000  // RPM
#define rotate_speed_limit 4000

#define PID_out_limit 6000
#define PID_I_limit   4000

#define Wheel_Diameter 152.0

#define Error_Limit 600

#define RPM_TO_mm_S  7.9587 //Wheel_Diameter*PI/60

int speed_PID_out_limit=120; //160


extern uint32_t system_time;   //系统时间 单位ms
extern volatile float yaw_angle,pitch_angle,roll_angle; //使用到的角度值
float balance_angle;
extern float Gyro_Balance,Gyro_Turn;

extern int RC[20];

//底盘控制任务
float CMspeed_kp=0,CMspeed_ki=0,CMspeed_kd=0;
float CM_Target_Speed[5],CM_Target_Speed_Last[5];
float CMspeedPID_P_out[5],CMspeedPID_I_out[5],CMspeedPID_D_out[5],CMspeedPID_out[5];
float e_speed[5],e_speed_last[5],e_speed_last_last[5],d_e_speed[5];


double target_angle=0,medie_angle=-10.1;  //-2.1
double target_speed=0,target_speed_last=0,rotate_speed=0;
int speed_PID_out,balance_PID_out,turn_PID_out;
int LEFT_PWM=0,RIGHT_PWM=0;
float left_encoder_speed,right_encoder_speed;

int wire_data[20],wire_data_flag;
float forward_leg_angle,back_leg_angle;
u8 imu_data_flag,imu_data_count,imu_data_heathy;
u8 remote_data_heathy;


u8 Robot_Move_Mode,Robot_Move_Mode_Last;
u8 Robot_Posture_Reset_Flag,Robot_Arm_Reset_Flag;
u8 Robot_leg_control_Flag;
u8 Robot_wheel_height_control_Flag;
u8 Camera_Platform_Control_Flag;
u8 Robot_Arm_Control_Flag;
int32_t  Temp_wheel_position;

u8 Push_Motor_Limit_Switch_Value;
int32_t Push_Motor_position;

Arm_Status_Data Arm;
volatile Arm_Rocker_Data Arm_Rocker;
extern Pad_App_Control_Data Pad_control_Data; 

u8 Up_Load_Data[80];
float Robot_Battery_Voltage,Robot_Battery_Current;

int Robot_Line_Speed,Robot_Rotate_Speed;
u8 Robot_Move_Enable;
int Front_Leg_speed,Back_Leg_speed;

float CM1_Distance,CM2_Distance,CM3_Distance,CM4_Distance;
float Robot_Move_Distance,Robot_Move_Distance_Last,Delta_Distance;
float Robot_Position_X,Robot_Position_Y;
float Robot_Yaw_Angel,Robot_Roll_Angel,Robot_Pitch_Angel;
float Line_Speed,Angl_Speed,Left_right_speed;
float Robot_Temp_Line_Speed,Robot_Temp_Angl_Speed;

union odometry  //
{
    float float_type;
    unsigned char char_type[4];
}odom_x,odom_y,odom_yaw,odom_line_vel,odom_angl_vel;     //

union IMU  //
{
    float float_type;
    unsigned char char_type[4];
}ACC_x,ACC_y,ACC_z,GY_x,GY_y,GY_z,QUAT_0,QUAT_1,QUAT_2,QUAT_3,Roll_Angle,Pitch_Angle,Yaw_Angle;

union ROS_Data_Receive  //
{
    float float_type;
    unsigned char char_type[4];
}Target_Line_Speed,Target_Angl_Speed,Target_Position_X,Target_Position_Y,Target_Angle,Target_left_right_speed;     //

float Ax, Ay, Az, Gx, Gy, Gz, Q0,Q1,Q2,Q3;

u8 ROS_Control_Data_Buf[30];
u8 ROS_Control_Data_Flag;
u8 ROS_Target_Velocity_Flag,ROS_Target_Position_Flag;
	
/*
*********************************************************************************************************
*                                            FUNCTIONS 
*********************************************************************************************************
*/


void LED_DUTY(int x)
{
	static int count=0;
	
	count++;
	
	  if(count==x)
	  {
	    GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_SET);	
			GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_SET);	
//			GPIO_SetBits(GPIOC, GPIO_Pin_1); //GREEN_LED_OFF();
			
			//send_data_pc();		
		}
		else
		if(count==2*x)
	  {
		  GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_RESET);	
			GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_RESET);	
//			GPIO_ResetBits(GPIOC, GPIO_Pin_1); //GREEN_LED_ON()      
			
			//USART_SendData(USART3, 0xaa);
      //while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);   //??????
		}
		
		if(count>=2*x)
			count=0;
}



/*************************************************************************************/
/*************************** CM-velocity-control ****************************************/
/*************************************************************************************/
//left_encoder_speed = 220 = 1.87 r/s = 1.65 m/s  ---- encoder one round : 8620 pulse
void CM_velocity_peoccess(void)
{
	static char cal_count=0;
	static int left_encoder,right_encoder,left_encoder_last,right_encoder_last;
	static int left_encoder_filter_buf[12],right_encoder_filter_buf[12],num=0;
	int i=0,left_sum=0,right_sum=0;
	
	cal_count++;
	
	//if(cal_count>2) //left_encoder_speed = 220 = 1.87 r/s = 1.65 m/s  ---- encoder one round : 8620 pulse
	if(cal_count>4)
	{
		cal_count=0;
		
		left_encoder_last = left_encoder;
		right_encoder_last = right_encoder;
		
		left_encoder = TIM3->CNT;
		right_encoder = TIM4->CNT;	
		
	//	left_encoder_speed = left_encoder - left_encoder_last;
	//	right_encoder_speed = right_encoder - right_encoder_last;
		
		left_encoder_filter_buf[num] = left_encoder - left_encoder_last;
		right_encoder_filter_buf[num] = right_encoder - right_encoder_last;
		
		if(left_encoder_filter_buf[num]>10000) 
		{
			left_encoder_filter_buf[num] = left_encoder_filter_buf[num] - 65535; 
		}	
		if(right_encoder_filter_buf[num]>10000) 
		{
			right_encoder_filter_buf[num] = right_encoder_filter_buf[num] - 65535; 
		}	
		if(left_encoder_filter_buf[num]<-10000) 
		{
			left_encoder_filter_buf[num] = left_encoder_filter_buf[num] + 65535; 
		}	
		if(right_encoder_filter_buf[num]<-10000) 
		{
			right_encoder_filter_buf[num] = right_encoder_filter_buf[num] + 65535; 
		}
		
		if(left_encoder_filter_buf[num]>500) left_encoder_filter_buf[num] = 500; //实测，150偏小，300比较合适
		if(right_encoder_filter_buf[num]>500) right_encoder_filter_buf[num] = 500;
		if(left_encoder_filter_buf[num]<-500) left_encoder_filter_buf[num] = -500; 
		if(right_encoder_filter_buf[num]<-500) right_encoder_filter_buf[num] = -500;
		
		num++;
		if(num>9)
		{
			num=0;
		}
		
		for(i=0;i<10;i++)
		{
			left_sum += left_encoder_filter_buf[i];
			right_sum += right_encoder_filter_buf[i];
		}
		
		left_encoder_speed = left_sum/10;
		right_encoder_speed = right_sum/10;
		
	}
	
//	USART_SendData(USART3, 0xAA);
//  while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);   //
	
//	USART_SendData(USART3, pitch_angle*10+0x80);
//  while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);   //Gyro_Balance
//	
//	USART_SendData(USART3, Gyro_Balance/10+0x80);
//  while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);   //Gyro_Balance
	
//	 USART_SendData(USART3, TIM3->CNT/512);
//   while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);   //
//	 
//	 USART_SendData(USART3, TIM4->CNT/512);
//   while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);   // 
	
}

void imu_data_healthy_judgement(void)
{
	static float robot_angle[6];
	static u8 imu_count=0;
	float tem;
	
	robot_angle[imu_count++] = pitch_angle;
	if(imu_count>4) imu_count=0;
	
	for(char i=0;i<5;i++)
	{
		for(char j=0;j<4-i;j++)
		{
			if(robot_angle[j]>robot_angle[j+1])
			{
				tem = robot_angle[j];
				robot_angle[j] = robot_angle[j+1];
				robot_angle[j+1] = tem;
			}
		}
	}
	
	//balance_angle = pitch_angle;
	balance_angle = robot_angle[2];
	
	if(imu_data_flag==0)
	{
		imu_data_count++;
		
		if(imu_data_count>100)
			imu_data_count=100;
	}
	else
	{
		imu_data_count=0;
	}
	
	if(imu_data_count>10)
	{
		imu_data_heathy = 0;
		//error_imu++;
	}
	else
	{
		imu_data_heathy = 1;
	}
	
	imu_data_flag=0;
}

/*************************************************************************************/
/*************************** CM-balance-control **************************************/
/*************************************************************************************/

void balance(float Angle,float Gyro)
{	
   float Bias=0,kp=-20,kd=-4.6; //-18 -4.5  
	
	 target_angle = 0;
//	
//	 if(target_angle>1&&e_speed[0]<-400)
//	 {
//		 target_angle = target_angle + target_angle*(e_speed[0]+400)/200; 
//	 }
//	 else if(target_angle<-1&&e_speed[0]>400)
//	 {
//		 target_angle = target_angle - target_angle*(e_speed[0]-400)/200; 
//	 }
	
	 //Bias = balance_angle - medie_angle + target_angle;       //===求出平衡的角度中值 和机械相关
	Bias = balance_angle;
	
	 //Bias = pitch_angle - medie_angle;       //===求出平衡的角度中值 和机械相关
	
	 //if(Bias>-0.2&&Bias<0.2)  Bias=0;
	
	 if(Bias>10)  Bias=10;
	 if(Bias<-10) Bias=-10;
	 
//	 if(Bias<0)
//	 {
//		 kp = kp*(1-(Bias+0)/15);
//		 kd = kd*(1-(Bias+0)/15);
//	 }
//	 else if(Bias>0)
//	 {
//		 kp = kp*(1+(Bias+0)/15);
//		 kd = kd*(1+(Bias+0)/15);
//	 }
	
//	 if(Bias>-0.2&&Bias<0.2)
//	 {
//		 CMspeedPID_I_out[0] = 0;
//	 }
	
	 if(Gyro_Balance>50)  Gyro_Balance=50;
	 if(Gyro_Balance<-50) Gyro_Balance=-50;	
	 //if(MPU6050_Real_Data.Gyro_Y>-4&&MPU6050_Real_Data.Gyro_Y<4)  MPU6050_Real_Data.Gyro_Y=0;
	 
	 balance_PID_out = kp*Bias + kd * Gyro_Balance;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 

	 if(balance_PID_out>200)  balance_PID_out=200;
	 if(balance_PID_out<-200) balance_PID_out=-200;
	
}



/*************************************************************************************/
/*************************** CM-turn-control *****************************************/
/*************************************************************************************/

void turn(int encoder_left,int encoder_right,float gyro)//转向控制
{
	
	turn_PID_out = 80 * 0 + 2 * Gyro_Turn;      //===结合Z轴陀螺仪进行PD控制
	
	if(turn_PID_out>50)  turn_PID_out=50;
	if(turn_PID_out<-50) turn_PID_out=-50;
	//turn_PID_out=0;
	
//	turn_PID_out_left = (rotate_speed - left_encoder_speed);
//	turn_PID_out_right = (-rotate_speed - right_encoder_speed);
//	
//	if(turn_PID_out_left>60) turn_PID_out_left=60;
//	if(turn_PID_out_left<-60) turn_PID_out_left=-60;
//	if(turn_PID_out_right>60) turn_PID_out_right=60;
//	if(turn_PID_out_right<-60) turn_PID_out_right=-60;
//	
//	if(wire_data[2]<-10 || wire_data[2]>10) 
//	{
//		turn_PID_out=0;
//	}
}

/*************************************************************************************/
/************************** Up_Load_Robot_Data *************************************/
/*************************************************************************************/

void Up_Load_Robot_Data(void)
{
	int i=0;
	unsigned char temp_buf[80];
	
	
	odom_x.float_type = Robot_Position_X;
	odom_y.float_type = Robot_Position_Y;
	odom_yaw.float_type = Robot_Yaw_Angel/180*PI;
	odom_line_vel.float_type = Robot_Temp_Line_Speed;
	odom_angl_vel.float_type = GY_z.float_type;
	
	Up_Load_Data[0] = 0x53;
	Up_Load_Data[1] = 0x49;
	
	for(i=0;i<4;i++)
	{
		temp_buf[i] = odom_x.char_type[i];
		temp_buf[i+4] = odom_y.char_type[i];
		temp_buf[i+8] = odom_yaw.char_type[i];
		temp_buf[i+12] = odom_line_vel.char_type[i];
		temp_buf[i+16] = odom_angl_vel.char_type[i];
		
		temp_buf[i+20] = ACC_x.char_type[i];
		temp_buf[i+24] = ACC_y.char_type[i];
		temp_buf[i+28] = ACC_z.char_type[i];
		temp_buf[i+32] = GY_x.char_type[i];
		temp_buf[i+36] = GY_y.char_type[i];
		temp_buf[i+40] = GY_z.char_type[i];
		temp_buf[i+44] = QUAT_0.char_type[i];
		temp_buf[i+48] = QUAT_1.char_type[i];
		temp_buf[i+52] = QUAT_2.char_type[i];
		temp_buf[i+56] = QUAT_3.char_type[i];
	}
	for(i=0;i<60;i++)
	{
		Up_Load_Data[i+2] = temp_buf[i];
	}  
	
	Up_Load_Data[62] = 0x41;
	
	for(u8 i=0;i<63;i++)
	{
//		USART_SendData(USART1, Up_Load_Data[i] );
//		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		
		USART_SendData(UART4, Up_Load_Data[i] );
		while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
		
	}
	
}

void Calculate_Robot_odometry(void)
{
	static float temp_data=0;
	CM1_Distance = -CM1Encoder.ecd_angle/180 * PI * Wheel_Diameter/19; // mm
	CM2_Distance = CM2Encoder.ecd_angle/180 * PI * Wheel_Diameter/19;
	CM3_Distance = CM3Encoder.ecd_angle/180 * PI * Wheel_Diameter/19;
	CM4_Distance = -CM4Encoder.ecd_angle/180 * PI * Wheel_Diameter/19;
	
	Robot_Move_Distance_Last = Robot_Move_Distance;
	Robot_Move_Distance = (CM1_Distance + CM2_Distance + CM3_Distance + CM4_Distance)/4;
	Delta_Distance = Robot_Move_Distance - Robot_Move_Distance_Last;
	
//	Delta_Distance=5;
//	temp_data+=0.36;
//	//if(temp_data>360) temp_data=0;
//	
//	Robot_Yaw_Angel = temp_data;
	
	Robot_Position_X += Delta_Distance * cos(Robot_Yaw_Angel/180*PI);
	Robot_Position_Y += Delta_Distance * sin(Robot_Yaw_Angel/180*PI);
	
	Robot_Temp_Line_Speed = 0.5*(CM2Encoder.RPM_rate - CM1Encoder.RPM_rate) * RPM_TO_mm_S;
	Robot_Temp_Angl_Speed = GY_z.float_type;
}

/*************************************************************************************/
/*************************** CM-speed-control *****************************************/
/*************************************************************************************/

void CMControlLoop(void)
{  
  ////////////////////remote_speed_control//////////////////////////////////////////////////
	
	if(Pad_control_Data.Control_Mode==0x01)
	{
		Line_Speed = 0.8*Pad_control_Data.Rocker_Y;
		Left_right_speed = 0.4*Pad_control_Data.Rocker_X;
		Angl_Speed = 0;
	}
	else
	{
		Line_Speed = 1000.0*Target_Line_Speed.float_type/RPM_TO_mm_S;	
		Left_right_speed = 1000.0*Target_Angl_Speed.float_type/RPM_TO_mm_S;	
		Angl_Speed = 100.0*Target_Angle.float_type;  // 3.14 rad/s == 310 RPM == 180 du/s
	}

	Line_Speed *=19;
	Angl_Speed *=19;
	Left_right_speed *=19;
	
	if(Line_Speed<-5) 
	{
		Line_Speed = Line_Speed + 5;
	}
	else if(Line_Speed<5)
	{
		Line_Speed = 0;
	} 
	else
	{
		Line_Speed = Line_Speed - 5;
	}
	if(Line_Speed>speed_ref_limit) Line_Speed = speed_ref_limit;
	if(Line_Speed<-speed_ref_limit) Line_Speed = -speed_ref_limit;
	
	if(Angl_Speed<-5) 
	{
		Angl_Speed = Angl_Speed + 5;
	}
	else if(Angl_Speed<5)
	{
		Angl_Speed = 0;
	} 
	else
	{
		Angl_Speed = Angl_Speed - 5;
	}
	if(Angl_Speed>speed_ref_limit) Angl_Speed = speed_ref_limit;
	if(Angl_Speed<-speed_ref_limit) Angl_Speed = -speed_ref_limit;
	
	if(Left_right_speed<-5) 
	{
		Left_right_speed = Left_right_speed + 5;
	}
	else if(Left_right_speed<5)
	{
		Left_right_speed = 0;
	} 
	else
	{
		Left_right_speed = Left_right_speed - 5;
	}
	if(Left_right_speed>speed_ref_limit) Left_right_speed = speed_ref_limit;
	if(Left_right_speed<-speed_ref_limit) Left_right_speed = -speed_ref_limit;
	
	
	//////////////////////////////////////////////////////////////////////////////////////////////////
	
	//target_speed = 0.005*target_speed + 0.995*target_speed_last;
	
//	if(blue_tooth_flag==6)
//	{
//		LED_DUTY(50);	
//		speed_line = vision_line_speed;
//		speed_angl = vision_angl_speed;
//	}
//	else if(blue_tooth_flag==1)
//	{
//		GPIO_WriteBit(GPIOF, GPIO_Pin_9, Bit_RESET);	
//	  speed_line = LINE;
//		speed_angl = 0;
//	}
//	else if(blue_tooth_flag==2)
//	{
//		GPIO_WriteBit(GPIOF, GPIO_Pin_9, Bit_SET);	
//	  speed_line = -LINE;
//		speed_angl = 0;
//	}
//	else if(blue_tooth_flag==3)
//	{
//		GPIO_WriteBit(GPIOF, GPIO_Pin_10, Bit_RESET);	
//	  speed_line = 0;
//		speed_angl = -ANGL;
//	}
//	else if(blue_tooth_flag==4)
//	{
//		GPIO_WriteBit(GPIOF, GPIO_Pin_10, Bit_SET);	
//	  speed_line = 0;
//		speed_angl = ANGL;
//	}
//	else 
//	{
//		GPIO_WriteBit(GPIOF, GPIO_Pin_9, Bit_RESET);	
//	  speed_line = 0;
//		speed_angl = 0;
//	}
	
	
  ////////////////////////////////////////////////////////////////////////////////////////
	
	CMspeed_kp = 8;  // 40
	CMspeed_ki = 0.06; // 1.1
	CMspeed_kd = 0;
	
	CM_Target_Speed_Last[1] = CM_Target_Speed[1];
	CM_Target_Speed_Last[2] = CM_Target_Speed[2];
	CM_Target_Speed_Last[3] = CM_Target_Speed[3];
	CM_Target_Speed_Last[4] = CM_Target_Speed[4];
	
	CM_Target_Speed[1] = -Line_Speed + Left_right_speed + Angl_Speed;
	CM_Target_Speed[2] = Line_Speed + Left_right_speed + Angl_Speed;
	CM_Target_Speed[3] = Line_Speed - Left_right_speed + Angl_Speed;
	CM_Target_Speed[4] = -Line_Speed - Left_right_speed + Angl_Speed;
	
	CM_Target_Speed[1] = 0.2*CM_Target_Speed[1] + 0.8*CM_Target_Speed_Last[1];
	CM_Target_Speed[2] = 0.2*CM_Target_Speed[2] + 0.8*CM_Target_Speed_Last[2];
	CM_Target_Speed[3] = 0.2*CM_Target_Speed[3] + 0.8*CM_Target_Speed_Last[3];
	CM_Target_Speed[4] = 0.2*CM_Target_Speed[4] + 0.8*CM_Target_Speed_Last[4];
	
	if(CM_Target_Speed[1]>speed_ref_limit) CM_Target_Speed[1]=speed_ref_limit;
	if(CM_Target_Speed[2]>speed_ref_limit) CM_Target_Speed[2]=speed_ref_limit;
	if(CM_Target_Speed[3]>speed_ref_limit) CM_Target_Speed[3]=speed_ref_limit;
	if(CM_Target_Speed[4]>speed_ref_limit) CM_Target_Speed[4]=speed_ref_limit;
	
	if(CM_Target_Speed[1]<-speed_ref_limit) CM_Target_Speed[1]=-speed_ref_limit;
	if(CM_Target_Speed[2]<-speed_ref_limit) CM_Target_Speed[2]=-speed_ref_limit;
	if(CM_Target_Speed[3]<-speed_ref_limit) CM_Target_Speed[3]=-speed_ref_limit;
	if(CM_Target_Speed[4]<-speed_ref_limit) CM_Target_Speed[4]=-speed_ref_limit;
	
//	e_speed[1] = 1.0*(CM_Target_Speed[1] - CM1Encoder.RPM_rate/19);
//	e_speed[2] = 1.0*(CM_Target_Speed[2] - CM2Encoder.RPM_rate/19);
//	e_speed[3] = 1.0*(CM_Target_Speed[3] - CM3Encoder.RPM_rate/19);
//	e_speed[4] = 1.0*(CM_Target_Speed[4] - CM4Encoder.RPM_rate/19);
	
	e_speed[1] = 1.0*(CM_Target_Speed[1] - CM1Encoder.RPM_rate);
	e_speed[2] = 1.0*(CM_Target_Speed[2] - CM2Encoder.RPM_rate);
	e_speed[3] = 1.0*(CM_Target_Speed[3] - CM3Encoder.RPM_rate);
	e_speed[4] = 1.0*(CM_Target_Speed[4] - CM4Encoder.RPM_rate);
	
	if(e_speed[1]>Error_Limit) e_speed[1]=Error_Limit;
	if(e_speed[2]>Error_Limit) e_speed[2]=Error_Limit;
	if(e_speed[3]>Error_Limit) e_speed[3]=Error_Limit;
	if(e_speed[4]>Error_Limit) e_speed[4]=Error_Limit;
	
	if(e_speed[1]<-Error_Limit) e_speed[1]=-Error_Limit;
	if(e_speed[2]<-Error_Limit) e_speed[2]=-Error_Limit;
	if(e_speed[3]<-Error_Limit) e_speed[3]=-Error_Limit;
	if(e_speed[4]<-Error_Limit) e_speed[4]=-Error_Limit;
 
	
	//CMspeedPID_I_out[0] += CMspeed_ki* (e_speed[0] + target_speed);
	
	CMspeedPID_P_out[1] += CMspeed_kp*(e_speed[1] - e_speed_last[1]);//CMspeedPID_P_out[1] =CMspeedPID_P_out[1] + CMspeed_kp*(e_speed[1] - e_speed_last[1]);
	CMspeedPID_I_out[1] += CMspeed_ki*e_speed[1];
	CMspeedPID_D_out[1] += CMspeed_kd*(e_speed[1]-2*e_speed_last[1]+e_speed_last_last[1]);
	
	if(CMspeedPID_I_out[1]<-PID_I_limit) CMspeedPID_I_out[1]=-PID_I_limit;
	if(CMspeedPID_I_out[1]> PID_I_limit) CMspeedPID_I_out[1]= PID_I_limit;
	
	CMspeedPID_P_out[2] += CMspeed_kp*(e_speed[2] - e_speed_last[2]);
	CMspeedPID_I_out[2] += CMspeed_ki*e_speed[2];
	CMspeedPID_D_out[2] += CMspeed_kd*(e_speed[2]-2*e_speed_last[2]+e_speed_last_last[2]);
	
	if(CMspeedPID_I_out[2]<-PID_I_limit) CMspeedPID_I_out[2]=-PID_I_limit;
	if(CMspeedPID_I_out[2]> PID_I_limit) CMspeedPID_I_out[2]= PID_I_limit;
	
	CMspeedPID_P_out[3] += CMspeed_kp*(e_speed[3] - e_speed_last[3]);
	CMspeedPID_I_out[3] += CMspeed_ki*e_speed[3];
	CMspeedPID_D_out[3] += CMspeed_kd*(e_speed[3]-2*e_speed_last[3]+e_speed_last_last[3]);
	
	if(CMspeedPID_I_out[3]<-PID_I_limit) CMspeedPID_I_out[3]=-PID_I_limit;
	if(CMspeedPID_I_out[3]> PID_I_limit) CMspeedPID_I_out[3]= PID_I_limit;
	
	CMspeedPID_P_out[4] += CMspeed_kp*(e_speed[4] - e_speed_last[4]);
	CMspeedPID_I_out[4] += CMspeed_ki*e_speed[4];
	CMspeedPID_D_out[4] += CMspeed_kd*(e_speed[4]-2*e_speed_last[4]+e_speed_last_last[4]);
	
	if(CMspeedPID_I_out[4]<-PID_I_limit) CMspeedPID_I_out[4]=-PID_I_limit;
	if(CMspeedPID_I_out[4]> PID_I_limit) CMspeedPID_I_out[4]= PID_I_limit;
	
	CMspeedPID_out[1] = CMspeedPID_P_out[1] + CMspeedPID_I_out[1] + CMspeedPID_D_out[1];
	CMspeedPID_out[2] = CMspeedPID_P_out[2] + CMspeedPID_I_out[2] + CMspeedPID_D_out[2];
	CMspeedPID_out[3] = CMspeedPID_P_out[3] + CMspeedPID_I_out[3] + CMspeedPID_D_out[3];
	CMspeedPID_out[4] = CMspeedPID_P_out[4] + CMspeedPID_I_out[4] + CMspeedPID_D_out[4];
	
	if(CMspeedPID_out[1]>PID_out_limit) CMspeedPID_out[1]=PID_out_limit;
	if(CMspeedPID_out[1]<-PID_out_limit)CMspeedPID_out[1]=-PID_out_limit;
	if(CMspeedPID_out[2]>PID_out_limit) CMspeedPID_out[2]=PID_out_limit;
	if(CMspeedPID_out[2]<-PID_out_limit)CMspeedPID_out[2]=-PID_out_limit;
	if(CMspeedPID_out[3]>PID_out_limit) CMspeedPID_out[3]=PID_out_limit;
	if(CMspeedPID_out[3]<-PID_out_limit)CMspeedPID_out[3]=-PID_out_limit;
	if(CMspeedPID_out[4]>PID_out_limit) CMspeedPID_out[4]=PID_out_limit;
	if(CMspeedPID_out[4]<-PID_out_limit)CMspeedPID_out[4]=-PID_out_limit;

	Speed_control(CAN2, CMspeedPID_out[1] , CMspeedPID_out[2] , CMspeedPID_out[3] ,CMspeedPID_out[4]);
	 
	e_speed_last_last[1] = e_speed_last[1];
	 e_speed_last_last[2] = e_speed_last[2];
	 e_speed_last_last[3] = e_speed_last[3];
	 e_speed_last_last[4] = e_speed_last[4];
	 
	 e_speed_last[1] = e_speed[1];
	 e_speed_last[2] = e_speed[2];
	 e_speed_last[3] = e_speed[3];
	 e_speed_last[4] = e_speed[4];
}

/*************************************************************************************/
/*************************** push_motor_control *****************************************/
/*************************************************************************************/

void Robot_wheel_height_control(void)
{
	//static u8 Limit_Switch_Reset=1;
	
	static int Position_Delta;
	
	if(Robot_wheel_height_control_Flag==1)
	{
		Position_Delta = RC[1]/20;
		
//		if(Push_Motor_Limit_Switch_Value==0x01)
//		{
//			if(Position_Delta>0) Position_Delta=0;	
//			
//			if(Limit_Switch_Reset==1)
//			{
//				Limit_Switch_Reset=0;
//				
//				Temp_wheel_position = 0;
//				
//				CAN_RoboModule_DRV_Reset(0,9);                      //对0组所有驱动器进行复位 
//				delay_ms(300);	
//				CAN_RoboModule_DRV_Mode_Choice(0,9,Position_Mode);  // 进入Position_Mode
//				delay_ms(200);			
//				CAN_RoboModule_DRV_Config(0,9,50,50);
//				delay_ms(200);	
//			}
//		}
//		else
//		{
//			Limit_Switch_Reset=1;
//		}

		if(my_fabs(balance_angle)>10)
		{
			if(Position_Delta>0) Position_Delta=0;	
		}
		
		Temp_wheel_position += Position_Delta;
		
		if(Temp_wheel_position<-90000) 
		{
			Temp_wheel_position = -90000;
		}
		if(Temp_wheel_position>90000) 
		{
			Temp_wheel_position = 90000;
		}
	}
	else
	{
		if(Arm_Rocker.Key6==1 && Arm_Rocker.Key5==1)
		{
			//if(my_fabs(balance_angle)<10)
			{
				Temp_wheel_position += 40;
			}
		}
		else if(Arm_Rocker.Key6==1 && Arm_Rocker.Key5==0)
		{		
			Temp_wheel_position += -40;		
		}
	}
	
	CAN_RoboModule_DRV_Position_Mode(0,9,5000,Temp_wheel_position);
}

/*************************************************************************************/
/*************************** leg_motor_control *****************************************/
/*************************************************************************************/

void Robot_Leg_Position_Control(void)
{
	get_leg_angle();
	
	if((forward_leg_angle<-40 && Front_Leg_speed>0) || (forward_leg_angle>80 && Front_Leg_speed<0))
	{
		Front_Leg_speed = 0;
	}
	
	if((back_leg_angle<-40 && Back_Leg_speed<0) || (back_leg_angle>80 && Back_Leg_speed>0))
	{
		Back_Leg_speed = 0;
	}
	
	Set_Leg_Speed(CAN2,Front_Leg_speed,Back_Leg_speed);
}

/*************************************************************************************/
/*************************** Track_control *****************************************/
/*************************************************************************************/

void Robot_Track_Control(void)
{
	static int Left_Track_Speed,Right_Track_Speed;
	//static int Line_Speed,Angle_Speed;
	
//	Line_Speed = -RC[1]/14;  //  800/14=60 RPM
//	Angle_Speed = -RC[3]/20; //  800/20=40 RPM
	
//	Left_Track_Speed = Robot_Line_Speed + Robot_Rotate_Speed ;
//	Right_Track_Speed = Robot_Line_Speed - Robot_Rotate_Speed ;
	
	Left_Track_Speed = Robot_Line_Speed/2 + Robot_Rotate_Speed/3 ;
	Right_Track_Speed = -Robot_Line_Speed/2 + Robot_Rotate_Speed/3 ;
	
	if(Left_Track_Speed>-5&&Left_Track_Speed<5) Left_Track_Speed=0;
	if(Right_Track_Speed>-5&&Right_Track_Speed<5) Right_Track_Speed=0;
	
	if(Left_Track_Speed>80) Left_Track_Speed = 80;
	if(Right_Track_Speed>80) Right_Track_Speed = 80;
	
	if(Left_Track_Speed<-80) Left_Track_Speed = -80;
	if(Right_Track_Speed<-80) Right_Track_Speed = -80;
	
	if(Robot_Move_Enable!=1)  // Robot_Move_Enable 
	{
		Left_Track_Speed = 0;
	  Right_Track_Speed = 0;
	}
	if(Robot_Move_Mode==2)  // Robot_Move_Enable 
	//if(Robot_Move_Enable==1)  // Robot_Move_Enable 
	{
		Set_Track_Speed(CAN2,Right_Track_Speed,Left_Track_Speed,Left_Track_Speed,Right_Track_Speed);
	}
	else
	{
		Set_Track_Speed(CAN2,0,0,0,0);
	}
}

/*************************************************************************************/
/*************************** control task *****************************************/
/*************************************************************************************/

void robot_reset_task(void)
{
	if(Robot_Posture_Reset_Flag==3)
	{
		//CAN_RoboModule_DRV_Position_Mode(0,1,5000,Temp_wheel_position);
	}
	
}

void remote_data_process_task(void)
{
	imu_data_healthy_judgement();
	
	Pad_Control_Data_Process();
	
	ROS_Control_Data_Process(); 
}


void balance_move_control_task(void)
{
	//CM_velocity_peoccess();
	balance(0,0);	
	turn(0,0,0);	
	CMControlLoop();
	
}

void track_move_control_task(void)
{
	//static uint8_t mode_change=0;
	
	Robot_wheel_height_control();
	
	Robot_Leg_Position_Control();
	
	Robot_Track_Control();
	
	
//	if(remote_data_heathy!=1)
//	{
//		Disable_Track_Driver(CAN2);
//		mode_change=1;
//	}
//	else if(wire_data[5]==3)
//	{
//		if(mode_change==1) 
//		{
//			mode_change=0;
//			Enable_Track_Driver(CAN2);
//			Enable_Track_Driver(CAN2);
//		}
//		move_by_track(wire_data[1],wire_data[2],0x43);
//		
//	}
//	else
//	{
//		Disable_Track_Driver(CAN2);
//		mode_change=1;
//	}
//	
	//LED_DUTY(100);	
}

void Arm_Control_Task(void)
{		
  //Pad_Control_Data_Process();
	
	//Arm_Rocker_Data_Process();
	
	Arm_Control();
	
	Camera_Platform_Control(); //
	
}


//控制任务，放在timer6 1ms定时中断中执行

u8 Robot_Control_Task_Flag,Data_process_Task_Flag;
void Control_Task(void)
{
	system_time++;
	
	CMControlLoop();
	
	LED_DUTY(50);			
}
/**********************************************************
*工作状态切换状态机,与1ms定时中断同频率
**********************************************************/


//云台pitch轴控制程序


void SetGimbalMotorOutput(void)
{
//	//云台控制输出								
//	//if((GetWorkState() == STOP_STATE) ||Is_Serious_Error() || GetWorkState() == CALI_STATE)   
//	if(0)   
//	{
//		Set_Gimbal_Current(CAN2, 0, 0);     //yaw + pitch			
//	}
//	else
//	{		
//		Set_Gimbal_Current(CAN2, -(int16_t)GMYSpeedPID.output, -(int16_t)GMPSpeedPID.output);     //yaw + pitch		
//    //Set_Gimbal_Current(CAN2, -(int16_t)yawPID_out, -(int16_t)pitchPID_out);     //yaw + pitch				
//	}		 
}
//控制任务初始化程序
void ControtLoopTaskInit(void)
{
	
}
