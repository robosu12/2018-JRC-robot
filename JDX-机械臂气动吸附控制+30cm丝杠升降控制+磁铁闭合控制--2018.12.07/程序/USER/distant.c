#include "distant.h"   
#include <math.h>


float Distant(double A_lat,double A_lon,double B_lat,double B_lon)
{
/*
	float afa,del,dis;
	afa=(A_lat-B_lat)*RAD;
	del=(A_lon-B_lon)*RAD;
	dis=2*asin(s000000000000000000000qrt(sin(afa/2)*sin(afa/2)+cos(A_lat*RAD)*cos(B_lat*RAD)*sin(del/2)*sin(del/2)))*EARTH_RAD;
	return dis;
	*/
	double x1,x2,y1,y2;

	x1=A_lat- B_lat;
	y1=A_lon- B_lon;

	x2=x1*60.0*1852.0;
	y2=y1*60.0*1852.0*cos(x1);

	return (sqrt(x2*x2+y2*y2));
		
}

float lat_distant(double A_lat,double B_lat)
{
	 double x1,x2;

	x1=A_lat- B_lat;
	x2=x1*60.0*1852.0;
	if(x2<0) x2=-x2;

	return x2;
}

float lon_distant(double A_lat,double A_lon,double B_lat,double B_lon)
{

	double x1,y1,y2;

	x1=A_lat- B_lat;
	y1=A_lon- B_lon;

	y2=y1*60.0*1852.0*cos(x1);
	if(y2<0) y2=-y2;
	return y2;
		
}





