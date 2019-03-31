#ifndef _LCD5110_H
#define _LCD5110_H
//#define    sce  PORTA_PA1  //片选引脚：芯片使能
//#define    res  PORTA_PA0  //复位引脚：外部复位输入端0复位
//#define    dc   PORTA_PA2  //数据/命令切换脚：1写数据，0写指令
//#define    sdin PORTA_PA3  //数据输入：串口数据输入端
//#define    sclk PORTA_PA4  //时钟引脚：串行时钟输入端
/*
#define    res=1    GPIO_WriteBit(GPIOE, GPIO_Pin_2, Bit_SET);//复位,0复位
#define    res=0    GPIO_WriteBit(GPIOE, GPIO_Pin_2, Bit_RESET);//复位,0复位

#define    sce=1    GPIO_WriteBit(GPIOE, GPIO_Pin_3, Bit_SET);//片选
#define    sce=0    GPIO_WriteBit(GPIOE, GPIO_Pin_3, Bit_RESET);//片选

#define    dc=1     GPIO_WriteBit(GPIOE, GPIO_Pin_4, Bit_SET);//1写数据，0写指令
#define    dc=0     GPIO_WriteBit(GPIOE, GPIO_Pin_4, Bit_RESET);//1写数据，0写指令

#define    sdin=1   GPIO_WriteBit(GPIOE, GPIO_Pin_5, Bit_SET);//数据
#define    sdin=0   GPIO_WriteBit(GPIOE, GPIO_Pin_5, Bit_RESET);//数据

#define    sclk=1   GPIO_WriteBit(GPIOE, GPIO_Pin_6, Bit_SET);//时钟
#define    sclk=0   GPIO_WriteBit(GPIOE, GPIO_Pin_6, Bit_RESET);//时钟
//sbit P27=P2^7;
 */

    

//6*16字符
//#pragma CODE_SEG DEFAULT
extern unsigned char shuzi[];
extern unsigned char  hanzi[];
extern unsigned char  zimu1[] ;
extern unsigned char  zimu2[] ;
extern unsigned char  fuhao[] ;
extern unsigned char F6x8[][6];

//void delayms(unsigned  int ii);
void LCD_write_byte(unsigned char dt, unsigned char command);
void LCD_init(void); //LCD初始化
void LCD_set_XY(unsigned char X, unsigned char Y);
void LCD_clear(void); //LCD清屏
void LCD_clearxy(unsigned char xLcd,unsigned char yLcd);
void LCD_write_shu(unsigned char* shuzi,unsigned char row, unsigned char page,unsigned char c);
void LCD_write_hanzi(unsigned char row, unsigned char page,unsigned char c);
 
 void LCD_tou(void);
 
 void LCD_P6x8Str(unsigned char xLcd,unsigned char yLcd,unsigned char ch[]);
 void LCD_P6x8num_i(unsigned char xLcd,unsigned char yLcd,int num);
 
//void LCD_WrDat(byte data);
// void LCD_WrCmd(byte cmd);
// void LCD_Set_Pos(byte x, byte y);
 
#endif