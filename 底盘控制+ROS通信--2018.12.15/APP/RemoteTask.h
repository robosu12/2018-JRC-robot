#ifndef _REOMTE_TASK_H_
#define _REOMTE_TASK_H_
#include "main.h"

/*
****************************************************************************
*
*																	MARCO
****************************************************************************
*/
//remote control parameters

#define RC_middle 1000

//RC_CtrlData
typedef struct{
	uint8_t rc_bytes[RC_FRAME_LENGTH];
}RC_Raw_t;

typedef __packed struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int8_t s1;
	int8_t s2;
}Remote;
typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
}Mouse;	
typedef	__packed struct
{
	uint16_t v;
}Key;

typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
}RC_Ctl_t;

typedef enum
{
	NOSHOOTING = 0,
	SHOOTING = 1,
}Shoot_State_e;



//ÊäÈëÄ£Ê½:Ò£¿ØÆ÷/¼üÅÌÊó±ê/Í£Ö¹ÔËÐÐ
typedef enum
{
	REMOTE_INPUT = 1,
	KEY_MOUSE_INPUT = 3,
	STOP = 2,
}InputMode_e;

//Ä¦²ÁÂÖ×´Ì¬Ã¶¾Ù
typedef enum
{
	FRICTION_WHEEL_OFF = 0,
	FRICTION_WHEEL_START_TURNNING = 1,
	FRICTION_WHEEL_ON = 2,
}FrictionWheelState_e;

//²¦¸Ë¶¯×÷Ã¶¾Ù
typedef enum
{
	FROM1TO2,
	FROM1TO3,
	FROM2TO1, 
	FROM3TO1,
	FROM3TO2,
}RC_SWITCH_ACTION_e;

//remote data process
typedef __packed struct
{
    int16_t forward_back_ref;
    int16_t left_right_ref;
    int16_t rotate_ref;
}ChassisSpeed_Ref_t;

//remote data process
typedef struct
{
    float pitch_angle_dynamic_ref;
    float yaw_angle_dynamic_ref;
    float pitch_angle_static_ref;
    float yaw_angle_static_ref;
    float pitch_speed_ref;
    float yaw_speed_ref;
}Gimbal_Ref_t;

//to detect the action of the switch
typedef struct RemoteSwitch_t
{
	 uint8_t switch_value_raw;            // the current switch value
	 uint8_t switch_value1;				  //  last value << 2 | value
	 uint8_t switch_value2;				  //
	 uint8_t switch_long_value; 		  //keep still if no switching
	 uint8_t switch_value_buf[4]; 
	 uint8_t buf_index;
	 uint8_t buf_last_index;
	 uint8_t buf_end_index;
}RemoteSwitch_t;

void SIYI_Remote_Data_Enable(void);
void Arm_Rocker_Data_Process(void);
void Pad_Control_Data_Process(void);
void Control_Platform_Data_Enable(void);
void Remote_Data_Enable(void);
void ROS_Control_Data_Process(void);


#endif

