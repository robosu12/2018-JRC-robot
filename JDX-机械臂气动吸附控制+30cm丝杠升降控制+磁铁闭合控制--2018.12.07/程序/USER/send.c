

#include "send.h"
#include "string.h"
#include "GPS.h"
#include "Lcd.h"
#include "systick.h"

char  snd_lolt[25]; //={"@3414.01910,10857.01139"};	//发送经纬度区，本身位置
extern uchar snd_ZigBee_stop;			  //声明外部变量不能赋值
extern uchar  snd_ZigBee_flag;
extern uchar GPS_Valid;			//有效接收GPS标志
extern uchar  ZigBee_Valid;		//有效接收ZigBee标志

extern GPS_INFO GPS;
extern int k;

void send_GPS()
{

}
void send_dir()
{

}


