#ifndef __GPS_H_
#define __GPS_H_

#include "stm32f10x.h"

#define uchar unsigned char
#define uint  unsigned int

typedef struct{
	int year;  
	int month; 
	int  day;
	int hour;
	int minute;
	int second;
}DATE_TIME;

typedef struct{
	char	str_lng[12];
	char 	str_lat[11];
	double 	lat_other;
	double 	lng_other;
	double  latitude;  //纬度
	double  longitude; //经度
	/*int     latitude_Degree;	//度
	int		latitude_Cent;		//分
	int   	latitude_Second;    //秒
	int     longitude_Degree;	//度
	int		longitude_Cent;		//分
	int   	longitude_Second;   //秒*/
	float 	speed;      //速度
	char 	ch_dir[7];
	float 	dir_other;
	float 	direction;  //航向
	float 	height_ground;    //水平面高度
	float 	height_sea;       //海拔高度
	u16     satellite;
	uchar 	NS;
	uchar 	EW;
	DATE_TIME D;
}GPS_INFO;

//void GPS_Init(void);
int GPS_RMC_Parse(char *line,GPS_INFO *GPS);
//int GPS_GGA_Parse(char *line,GPS_INFO *GPS);
//int GPS_GSV_Parse(char *line,GPS_INFO *GPS);
int GPS_OTH_Parse(char *line,GPS_INFO *GPS);
void Int_To_Str(int x,char *Str);

#endif  //__GPS_H_
