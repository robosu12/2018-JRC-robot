#ifndef __DISTANT_H__
#define __DISTANT_H__ 

#define		RAD			(3.1415926/180)
#define		EARTH_RAD	6378.137
float Distant(double A_lat,double A_lon,double B_lat,double B_lon);
float lat_distant(double A_lat,double B_lat);
float lon_distant(double A_lat,double A_lon,double B_lat,double B_lon);

#endif
