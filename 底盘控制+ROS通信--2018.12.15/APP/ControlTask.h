#ifndef _CONTROL_TASK_H_
#define _CONTROL_TASK_H_
#include "main.h"


//initiate status: 
typedef enum
{
    PREPARE_STATE,     		//上电后初始化状态 4s钟左右
    STANDBY_STATE,			//云台停止不转状态
    NORMAL_STATE,			//无输入状态
    STOP_STATE,        	//停止运动状态
    CALI_STATE,    			//校准状态
}WorkState_e;

typedef struct
{
	u8 Control_Mode;
	u8 Control_Object;
	int Rocker_X;
	int Rocker_Y;
	
	u8 Robot_Plat_Control_Object;
	u8 Arm_Sig_Joint_Control_Object;
	u8 Arm_Link_Control_Object;

}Pad_App_Control_Data;

void Control_Task(void);

void LED_DUTY(int x);
void balance(float Angle,float Gyro);
void Robot_wheel_height_control(void);
void turn(int encoder_left,int encoder_right,float gyro);
void CM_velocity_peoccess(void);
void imu_data_healthy_judgement(void);
void Calculate_Robot_odometry(void);
void CMControlLoop(void);

void remote_data_process_task(void);
void balance_move_control_task(void);
void track_move_control_task(void);
void Arm_Control_Task(void);

void Up_Load_Robot_Data(void);

#endif

