#ifndef _CAN1_BUS_TASK_H_
#define _CAN1_BUS_TASK_H_

#include "main.h"

/* CAN Bus 1 */  
#define OpenLoop_Mode                       0x01
#define Current_Mode                        0x02
#define Velocity_Mode                       0x03
#define Position_Mode                       0x04
#define Velocity_Position_Mode              0x05
#define Current_Velocity_Mode               0x06
#define Current_Position_Mode               0x07
#define Current_Velocity_Position_Mode      0x08

#define UP_POSITION       0X1E848   // 125000
#define MIDDLE_POSITION   0X0FDE8   // 65000
#define DOWN_POSITION     0X01388   // 5000

#define FULL_PWM          0X01388   // 5000

#define SPEED_SCALE 50

int my_abs(int x);
float my_fabs(float x);
	
static void CAN_Delay_Us(unsigned int t);
void Limit_Switch_process(CanRxMsg * msg);
void Can1ReceiveMsgProcess(CanRxMsg * msg);
void CAN_RoboModule_DRV_Reset(unsigned char Group,unsigned char Number);
void CAN_RoboModule_DRV_Mode_Choice(unsigned char Group,unsigned char Number,unsigned char Mode);
void CAN_RoboModule_DRV_OpenLoop_Mode(unsigned char Group,unsigned char Number,short Temp_PWM);
void CAN_RoboModule_DRV_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity);
void CAN_RoboModule_DRV_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,long Temp_Position);
void CAN_RoboModule_DRV_Config(unsigned char Group,unsigned char Number,unsigned char Temp_Time1,unsigned char Temp_Time2);

void Arm_Motor_Driver_Init(void);
void Robo_Module_Speed_Set(uint8_t node_no,int Temp_Velocity); //
void Robo_Module_Current_Set(uint8_t node_no,int Temp_PWM,int Temp_Current);

#endif

