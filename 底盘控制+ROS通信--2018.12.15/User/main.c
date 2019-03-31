#include "main.h"
	
uint32_t Upload_Speed = 1;   //数据上传速度  单位 Hz
#define upload_time (1000000/Upload_Speed)  //计算上传的时间。单位为us

uint32_t system_time;   //系统时间 单位ms
uint32_t time_count=0;

extern volatile float yaw_angle,pitch_angle,roll_angle; //使用到的角度值
extern float Gyro_Balance,Gyro_Turn;
extern float forward_leg_angle,back_leg_angle;
extern u16 left_encoder_send[8],right_encoder_send[8];
extern int error_imu,error_remote;

extern int RC[20];
extern uint16_t remote_channel_data[20];

extern u8 Limit_Switch_Value;
extern int32_t  Temp_wheel_position;
extern Arm_Status_Data Arm;
extern volatile Arm_Rocker_Data Arm_Rocker;

extern u8 Robot_Move_Mode,Robot_Move_Mode_Last;
extern int Robot_Line_Speed,Robot_Rotate_Speed;
extern  u8 Robot_Move_Enable;
extern int Front_Leg_speed,Back_Leg_speed;
extern int Yaw_Speed,Ptich_Speed;
extern u8 Robot_Posture_Reset_Flag,Robot_Arm_Reset_Flag;

extern float CM1_Distance,CM2_Distance,CM3_Distance,CM4_Distance,Robot_Move_Distance;
extern float Robot_Yaw_Angel,Robot_Roll_Angel,Robot_Pitch_Angel;

int main(void)
{     			
	//ControtLoopTaskInit();   //app init
	//RemoteTaskInit();
	//delay_ms(800); 
  delay_ms(1000);
	BSP_Init();	
	//delay_ms(20);	
	//system_micrsecond = Get_Time_Micros();				
	while(1)
	{  			
		if(time_count!=system_time)
		{
			time_count=system_time;				
			
			if(system_time%2==0)
			{
				remote_data_process_task();	
				Calculate_Robot_odometry();		
			}
			
			if(system_time%10==0)
			{
				//send_data_pc();
				Up_Load_Robot_Data();
			}
		}	
		
//		if(time_count%4==0)
//		{
//			remote_data_process_task();
//			balance_move_control_task();	//1ms
//      track_move_control_task();    //1ms	
//			Arm_Control_Task();           //2ms	
//		}
  }
}

void send_data_pc(void)
{
	//printf(" robot data : \n");	
	//printf(" pitch_angle : %3.2f , Gyro_Balance : %3.2f , Gyro_Turn : %3.2f ; \n",pitch_angle,Gyro_Balance,Gyro_Turn);
//	printf(" remote_data - 0 : %d ,1 : %d ,2 : %d ,3 : %d ,",RC[0],RC[1],RC[2],RC[3]);
//	printf(" 4 : %d ,5 : %d ,6 : %d ,7 : %d ,",RC[4],RC[5],RC[6],RC[7]);	//left_encoder_send[8]
//	printf(" 8 : %d ,9 : %d ,10 : %d ,11 : %d ,",RC[8],RC[9],RC[10],RC[11]);	
//	printf(" 12 : %d ,13 : %d ,14 : %d ,15 : %d \n",RC[12],RC[13],RC[14],RC[15]);
	
	//printf(" left_encoder : %d , right_encoder : %d \n",left_encoder_send[0],right_encoder_send[0]);	//left_encoder_send[8]
	//printf(" Temp_wheel_position : %d \n",-Temp_wheel_position/1000);
	
//	printf(" -Balance_angle : %3.2f ,Gyro : %3.2f \n",balance_angle , Gyro_Balance); 
//	
//	printf(" -forward_leg_angle : %3.1f , back : %3.1f \n",forward_leg_angle , back_leg_angle);
//	
//	printf(" -Arm_Angle 1 : %3.1f , 2 : %3.1f , 3 : %3.1f , 4 : %3.1f \n",Arm.Joint_1_Angle,Arm.Joint_2_Angle,Arm.Joint_3_Angle,Arm.Joint_4_Angle);
	
//  printf(" -Joint_3_X  : %3.4f , Y : %3.4f \n",Arm.Joint_3_Coordinate.X ,Arm.Joint_3_Coordinate.Y);
//	printf(" -Hold_Angle : %3.1f \n",Arm.Terminal_Hold_Angle);
	
//	printf(" -%d , %d , %d , %d , %d , %d ",Arm_Rocker.Key1,Arm_Rocker.Key2,Arm_Rocker.Key3,Arm_Rocker.Key4,Arm_Rocker.Key5,Arm_Rocker.Key6);
//	printf(" -%d , %d , %d , %d , %d ",Arm_Rocker.X,Arm_Rocker.Y,Arm_Rocker.Z1,Arm_Rocker.Z2,Arm_Rocker.Speed_Adjust);
//	printf(" -%d , %d , %d , %d ",Robot_Move_Mode,Robot_Move_Enable,Robot_Line_Speed,Robot_Rotate_Speed);
//	printf(" -%d , %d , %d , %d , %d , %d \n",Front_Leg_speed,Back_Leg_speed,Yaw_Speed,Ptich_Speed,Robot_Arm_Reset_Flag,Robot_Posture_Reset_Flag);

	//printf("-yaw_angle : %3.2f ,Gyro : %3.2f \n",yaw_angle , Gyro_Turn); 
	//printf("-Robot_Move_Distance : %2.3f , CM1 : %2.3f , CM2 : %2.3f , CM3 : %2.3f , CM4 : %2.3f ; \n",Robot_Move_Distance,CM1_Distance,CM2_Distance,CM3_Distance,CM4_Distance);
	
	printf("-Roll : %3.2f , Pitch : %3.2f , Yaw : %3.2f \n",Robot_Roll_Angel , Robot_Pitch_Angel,Robot_Yaw_Angel);

//			
//			USART_SendData(UART4, error_imu);//max:AC min:46
//			while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);   //forward_leg_angle
//			
//			USART_SendData(UART4, error_remote);//max:A8    min:47
//			while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);   //forward_leg_angle
}
