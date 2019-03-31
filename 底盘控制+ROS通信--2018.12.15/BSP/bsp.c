#include "main.h"

void BSP_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);     
	//PWM_Configuration();
	//PWM_Out_Init(10000);    
	//TIM2_Configuration();		 
	
	//Quad_Encoder_Configuration();	
	
	Led_Configuration(); 

  Adc_Init();	
	
	CAN1_Configuration();           
	CAN2_Configuration();
	        
	USART3_Configuration(); 
	
	TIM6_Configuration();		
	TIM6_Start();   	
	
	//MPU6050_IntConfiguration();     
	//MPU6050_EnableInt();    	
	//Encoder_Start();
	
//	Arm_Motor_Driver_Init();
//	
//	Enable_Balance_Driver(CAN2);  // track motor enable ; balance motor enable
//	Enable_Balance_Driver(CAN2);
//	
//	Enable_Track_Driver(CAN2);
//	Enable_Track_Driver(CAN2);
//	
//	Enable_Leg_Driver(CAN2);
//	Enable_Leg_Driver(CAN2);
	
	
//  CAN_RoboModule_DRV_Reset(0,0);                      //对0组所有驱动器进行复位 
//	delay_ms(300);	
//  CAN_RoboModule_DRV_Mode_Choice(0,9,Position_Mode);  //0组的1驱动器 进入Position_Mode
//	delay_ms(200);	
//	CAN_RoboModule_DRV_Config(0,9,0,50);
//	delay_ms(200);
}

