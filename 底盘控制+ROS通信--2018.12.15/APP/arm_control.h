#ifndef __ARM_CONTROL_H
#define __ARM_CONTROL_H

#include "main.h"

#define SCALE1  (66*40)//减速比    5500 RPM
#define SCALE2  (126*36)//126*40   7000 RPM
#define SCALE3  (66*38)//          5500 RPM
#define SCALE4  (159*32)//         5200 RPM
//3800
//3800

#define REDUCTION_JOINT_1  (100*7.5)//减速比
#define REDUCTION_JOINT_2  2772//(308*9)//(90*126)
#define REDUCTION_JOINT_3  (257*9)//
#define REDUCTION_JOINT_4 (257*1.5)//
  
#define PI      3.1415926
#define L2      0.49  // length of big arm(m)
#define L3      0.398 // length of small arm(m)
//#define L4      0.233 
#define L4      0.488

#define ENCRES              512      //编码器线数
#define Joint_2_offset_Angle  16.48   //16.48
#define Joint_3_offset_Angle  28.95   //16.48+12.47 = 28.95
//#define Joint_4_Reset_Angle  19.63   //12.47+7.16 = 19.63
#define Joint_4_offset_Angle  15.88   //12.47+3.41 = 19.63



typedef struct
{
	float X; //  (m)	
	float Y; //  (m)	
	float Z; //  (m)

}Coordinate;

typedef struct
{
	unsigned char Key1;  //1-on 0-off;sw1
	unsigned char Key2;  //sw2
	
	unsigned char Key3;  //sw3
	unsigned char Key4;  //sw4
	unsigned char Key6;	 //sw6
	
	unsigned char Key5;  //sw5
	
	int Speed_Adjust;

	int X; //  (0-255)	
	int Y; //  (0-255)	
	int Z1; // (0-255)
	int Z2; // (0-255)

}Arm_Rocker_Data;

typedef struct
{
	Coordinate Joint_2_Coordinate;
	Coordinate Joint_3_Coordinate;
	Coordinate Joint_4_Coordinate;
	Coordinate Joint_6_Coordinate;
	
	int Joint_1_Position; 
	int Joint_2_Position;
	int Joint_3_Position;
	int Joint_4_Position;
	
	int Joint_1_Reset_Position;
	int Joint_2_Reset_Position;
	int Joint_3_Reset_Position;
	int Joint_4_Reset_Position;
	
	float Joint_1_Angle;
	float Joint_2_Angle;
	float Joint_3_Angle;
	float Joint_4_Angle;
	float Terminal_Hold_Angle;

	unsigned char Joint_1_Limit_Switch;
	unsigned char Joint_2_Limit_Switch;
	unsigned char Joint_3_Limit_Switch;
	unsigned char Joint_4_Limit_Switch;

}Arm_Status_Data;


int Get_Joint_Motor_Pos(unsigned char num);
float Get_Joint_Angle(unsigned char num,int d);
void Get_All_Joint_Angle_And_Coordinate(void);

void Set_Arm_Joint_Angular_Velocity(u8 Joint,float Angular_Velocity);
void Set_Arm_Joint_Motor_Rotate_Speed(uint8_t Joint,int Rotate_Speed);

void Arm_Joint_Control_Special(void);
void Arm_Joint_Control(void);
void Arm_Link_Control(void);
void Arm_Auto_Stretch_And_Reset(void);
void Arm_Stop(void);

void Arm_Control(void);
	
void Camera_Platform_Control(void);


#endif 


