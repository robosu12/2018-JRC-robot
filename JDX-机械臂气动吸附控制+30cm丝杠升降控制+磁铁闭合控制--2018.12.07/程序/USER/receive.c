
#include "receive.h"
#include "GPS.h"
#include "string.h"
#include "misc.h"
#include "lcd.h"
#include "touch.h"
#include "distant.h"
#include "time_test.h"
#include "speech.h"

char  warning[]={"警告！！"};
char  warn_msg[]={"前方    米有停车"};
char  warn_hlp[]={"前方    米需帮忙"};
char  warn_msg_sp[]={"前方有停车！"};
char  warn_hlp_sp[]={"前方车辆需帮忙！"};
char  warn_msg_near[]={"小心有停车！"};
char  warn_hlp_near[]={"附近车辆需帮忙！"};
char  warn_msg_cross[]={"小心横向车辆"};

extern char  init4[];
extern GPS_INFO   GPS;  //在display.c中定义，使用时要加extern
extern char rev_buf[80];
extern char rev_lolt[26];
extern uchar gps_flag;      //GPS处理标志
extern uchar volatile rev_GPS_stop;     //接收GPS停止标志
extern uchar volatile rev_ZigBee_stop;	//接收ZigBee结束标志
extern uchar volatile rev_ZigBee_start;
extern uchar rev_ZigBee_flag;		//处理ZigBee数据
extern uchar volatile Park_rq;
extern uchar volatile GPS_num;
//extern uchar volatile time_warn;
extern uchar volatile Help_rq;
extern uchar volatile cross_wr;
extern uchar volatile ZigBee_num;

extern double speed,longitude,latitude,direct;
extern char direction[];
extern char direction_ch[];
extern int flag_CE,flag_CL;
extern uchar NS,EW;


uchar  GPS_Valid=0;			//有效接收GPS标志
uchar  ZigBee_Valid=0;		//有效接收ZigBee标志
extern u8 warn_flag;
extern u8 speed_show;
extern u8 flag_flag;
int GPS_TIME=0;

void delay2(int x)
{
 int i,j;
 for(i=0;i<1000;i++)
 for(j=0;j<x;j++);
}



void receive_RMC()
{
	
		GPS_Valid= GPS_RMC_Parse(rev_buf, &GPS);
	
		if (GPS_Valid) //解析GPRMC
		{
			speed=GPS.speed;
			longitude=GPS.longitude/100;	//缺东西经
			latitude=GPS.latitude/100;
			direct=GPS.direction;
			NS=GPS.NS;
			EW=GPS.EW;
			
			GPS_TIME++;	  
		
		}
		
	gps_flag = 0;
	rev_GPS_stop  = 0;	
}


void receive_ZigBee()		   //接收一帧数据要1.04ms
{

 }
