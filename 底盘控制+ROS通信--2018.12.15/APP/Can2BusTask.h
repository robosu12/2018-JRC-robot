#ifndef _CAN2_BUS_TASK_H_
#define _CAN2_BUS_TASK_H_

#include "main.h"

/* CAN Bus 1 */  
#define CAN_BUS1_ZGYRO_FEEDBACK_MSG_ID            0x401

#define CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID           0x201
#define CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID           0x202 
#define CAN_BUS2_MOTOR3_FEEDBACK_MSG_ID           0x203
#define CAN_BUS2_MOTOR4_FEEDBACK_MSG_ID           0x204
#define CAN_BUS2_MOTOR5_FEEDBACK_MSG_ID           0x205
#define CAN_BUS2_MOTOR6_FEEDBACK_MSG_ID           0x206

#define CAN_BUS1_ZGYRO_FEEDBACK_MSG_ID   		  0x401

extern volatile Encoder CM1Encoder;
extern volatile Encoder CM2Encoder;
extern volatile Encoder CM3Encoder;
extern volatile Encoder CM4Encoder;

void EncoderProcess(volatile Encoder *v, CanRxMsg * msg);
void EncoderProcess_HaErBin(volatile Encoder *v, CanRxMsg * msg);
void Can2ReceiveMsgProcess(CanRxMsg * msg);

void Speed_control(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);

void Set_Balance_Speed(CAN_TypeDef *CANx, int32_t cm1_iq, int32_t cm2_iq);
void Get_Balance_Speed(CAN_TypeDef *CANx);
void Set_Track_Speed(CAN_TypeDef *CANx, int32_t cm1_iq, int32_t cm2_iq, int32_t cm3_iq, int32_t cm4_iq);
void Get_Track_Speed(CAN_TypeDef *CANx);
void Set_Leg_Speed(CAN_TypeDef *CANx, int32_t Front_Leg, int32_t Back_Leg);

void Enable_Balance_Driver(CAN_TypeDef* CANx);
void Disable_Balance_Driver(CAN_TypeDef* CANx);
void Enable_Track_Driver(CAN_TypeDef* CANx);
void Disable_Track_Driver(CAN_TypeDef* CANx);
void Enable_Leg_Driver(CAN_TypeDef* CANx);


#endif

