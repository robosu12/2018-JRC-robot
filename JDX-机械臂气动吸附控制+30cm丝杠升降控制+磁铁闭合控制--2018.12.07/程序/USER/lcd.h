	  /***********************************************************
                       野火LCD命名规则
 默认为横屏，英文默认为8*16，汉字默认为16*16。字符串默认左对齐。

 各缩写:
 LCD	液晶		Liquid Crystal Display
 Char	字符		character						这里表示一个字的意思，显示一个汉字，也是用Char标识
 Str	字符串		string
 Num	数字		Number
 CH		中文		Chinese
 ENCH	中英混显	English-Chinese
 O		叠加效果	overlying
 P		竖屏		Portrait
 6x12	字体大小为6*12
 R		右对齐		right

 排列顺序:
 		类型		大小	   语言属性	   显示选项   对齐方式 	  屏幕选项
 LCD----Str 	----6x12	----CH		----O		----R		----P
        Char					ENCH	
 		Num
        XXX				

 这里XXX表示Point、Line、Rectangle等等。
 同一列表示同等优先顺序。
 部分参数可不要
***********************************************************/


/***********************************************************
 				野火LCD函数参数定义顺序规则
 (x,y)	----str		----其他未知属性	----(color,bkColor)
			char		如:len
			num
***********************************************************/

#ifndef __LCD_H
#define	__LCD_H

#include "stm32f10x.h"
#include "lcd_botton.h"

/******常用颜色*****/
#define RED	  0XF800
#define GREEN 0X07E0
#define BLUE  0X001F  
#define BRED  0XF81F
#define GRED  0XFFE0
#define GBLUE 0X07FF
#define BLACK 0X0000
#define WHITE 0XFFFF
#define GRAY  0XC0C0

void LCD_Init(void);

/***************  LCD绘画  ***************/
void LCD_Point			(u16 x,u16 y);
void LCD_ColorPoint		(u16 x,u16 y,u16 rgb565); 
void LCD_Line			(u16 x1,u16 y1,u16 x2,u16 y2);
void LCD_yuan           (int Clo);
void LCD_Rectangle		(u16 x,u16 y,u16 len,u16 wid,u16 rgb565);					/*矩形*/
void LCD_led            (int Clo);

/***************横屏字符显示  Landscape横向***************/
void LCD_Char			(u16 x,u16 y,	u8 ascii,	u16 Color,u16 bkColor);			/*8*16*/
void LCD_Char_O			(u16 x,u16 y,	u8 acsii,	u16 Color);
void LCD_Char_CH		(u16 x,u16 y,const u8 *str,	u16 Color,u16 bkColor);
void LCD_Char_CH_O		(u16 x,u16 y,const u8 *str,	u16 Color);
void LCD_Char_12x24_O(u16 x, u16 y,u8 acsii,u16 Color);      /*12*24 ascii*/  

void LCD_Str_O			(u16 x,u16 y,const u8 *str,	u16 Color); 					/*8*16 ascii*/
void LCD_Str_CH			(u16 x,u16 y,const u8 *str,	u16 Color,u16 bkColor); 		/*16*16汉字*/
void LCD_Str_CH_O		(u16 x,u16 y,const u8 *str,	u16 Color);	        			/*16*16汉字*/


//void LCD_Num_8x16_O_P(u16 x,u16 y,u32 num, u16 Color);
void LCD_Num_6x12_O		(u16 x,u16 y,	u32 num,	u16 Color);	   			 		/*6*12 */
void LCD_Num_12x24_O(u16 x,u16 y,u32 num, u16 Color);
void LCD_Char_6x12_O	(u16 x,u16 y,	u8 acsii,	u16 Color);						//6*12
void LCD_Str_6x12_O		(u16 x,u16 y,const u8 *str,	u16 Color);	 					/*6*12 ascii*/
void LCD_Str_12x24_O(u16 x, u16 y,const u8 *str,u16 Color);       /*12*24 ascii*/  
void LCD_Num_8x16_O(u16 x,u16 y,u32 num, u16 Color);
void LCD_Char_8x16_O(u16 x, u16 y, u8 acsii,u16 Color);


/***************竖屏字符显示  Portrait纵向***************/

void LCD_Char_O_P		(u16 x,u16 y,	u8 acsii,	u16 Color);
void LCD_Char_CH_P		(u16 x,u16 y,const u8 *str,	u16 Color,u16 bkColor);
void LCD_Char_CH_O_P	(u16 x,u16 y,const u8 *str,	u16 Color);

void LCD_Str_O_P		(u16 x,u16 y,const u8 *str,	u16 Color); 					/*8*16 ascii*/			
void LCD_Str_CH_P		(u16 x,u16 y,const u8 *str,	u16 Color,u16 bkColor); 		/*16*16汉字*/
void LCD_Str_CH_O_P		(u16 x,u16 y,const u8 *str,	u16 Color);	    				/*16*16汉字*/

void LCD_Char_6x12_O_P	(u16 x,u16 y,	u8 acsii,	u16 Color);						//6*12
void LCD_Str_6x12_O_P	(u16 x,u16 y,const u8 *str,	u16 Color);	  					/*6*12 ascii*/	



/***************为其他实验专门编写的函数***************/
//计算器实验专用
void LCD_Str_R			(u16 x,u16 y,const u8 *str,	u8 len,u16 Color,u16 bkColor); 	/*右对齐，超过len位，仅显示最后len位*/

//MP3实验用
void LCD_Str_ENCH_O_P	(u16 x,u16 y,const u8 *str,	u16 Color);	     				/*中英文混显*/



#endif /* __LCD_H */

