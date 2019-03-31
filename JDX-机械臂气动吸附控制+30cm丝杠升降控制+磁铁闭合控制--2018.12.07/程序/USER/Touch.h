
	   #ifndef _TOUCHPANEL_H_
#define _TOUCHPANEL_H_

#include "stm32f10x.h"


typedef	struct POINT 
{
   uint16_t x;		
   uint16_t y;
}Coordinate;


typedef struct Parameter 
{						
long double An,  			 //注:sizeof(long double) = 8
            Bn,     
            Cn,   
            Dn,    
            En,    
            Fn,     
            Divider ;
}Parameter ;




extern Coordinate ScreenSample[4];
extern Coordinate DisplaySample[4];
extern Parameter touch_para ;
extern Coordinate  display ;



/* AD通道选择命令字和工作寄存器 */
#define	CHX 	0xd0 	/* 通道Y+的选择控制字 */	
#define	CHY 	0x90		/* 通道X+的选择控制字 */



#define CS_2046(a)	if (a)	\
					GPIO_SetBits(GPIOB,GPIO_Pin_7);\
					else		\
					GPIO_ResetBits(GPIOB,GPIO_Pin_7)



#define INT_IN_2046   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6)

#define RED   0XF800
#define GREEN 0X07E0
#define BLUE  0X001F  
#define BRED  0XF81F
#define GRED  0XFFE0
#define GBLUE 0X07FF
#define BLACK 0X0000
#define WHITE 0XFFFF
#define PURPUE 0XA294
#define GRAY   0XC0C0
#define CAL_BACKGROUND_COLOR 	PURPUE

#define shuzi_color GBLUE

			
void Touch_init(void);

char  Get_touch_place(u16 *x, u16 *y);
void TOUCH_NVIC_Configuration(void);	
Coordinate *Read_2046(void);
Coordinate *Read_2046_2(void);
int Touchl_Calibrate(void);
void DrawCross(uint16_t Xpos,uint16_t Ypos);
int Palette_draw_point(uint16_t Xpos,uint16_t Ypos);
FunctionalState Cal_touch_para( Coordinate * displayPtr,Coordinate * screenPtr,Parameter * para);
FunctionalState Get_touch_point(Coordinate * displayPtr,Coordinate * screenPtr,Parameter * para );
void Init_Palette(void);

#endif
