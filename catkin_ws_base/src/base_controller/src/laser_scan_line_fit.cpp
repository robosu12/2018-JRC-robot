#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <math.h>
using namespace std;

float angle_min;
float angle_max;
float angle_increment;
float range_min;
float range_max;
//只取+-30度的数据
float ranges[61];
float position_x;
float position_y;

int table_index=0;
float target_angle_start=-0.26;

sensor_msgs::LaserScan mod_scan_data;
int receive_flag=0;
int angle_num=41;

float lineFit(int column);
void laser_scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg );


void laser_scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg )
{
    receive_flag=1;

    vector<float> temp_range;
    angle_min = msg->angle_min;
    angle_max = msg->angle_max;
    angle_increment = msg->angle_increment;

    ros::Time scan_time = ros::Time::now();
    //angle_num = 2.0*fabs(target_angle_start) / msg->angle_increment;

    mod_scan_data.header.stamp = scan_time;
    mod_scan_data.header.frame_id = "laser_link";
    mod_scan_data.range_min = 0.05;
    mod_scan_data.range_max = 20;
    mod_scan_data.angle_min = -msg->angle_increment*(angle_num-1)/2;
    mod_scan_data.angle_max =  msg->angle_increment*(angle_num-1)/2;
    mod_scan_data.angle_increment = msg->angle_increment;
    mod_scan_data.ranges.resize(angle_num);
    mod_scan_data.intensities.resize(angle_num);

    for (int i = 0;i< angle_num;i++)
    {
        //temp_range.push_back(msg->ranges[i + (target_angle_start-angle_min)/msg->angle_increment ]);
        mod_scan_data.ranges[i] = msg->ranges[i + (mod_scan_data.angle_min - msg->angle_min) / msg->angle_increment ];
        mod_scan_data.intensities[i] = 100;
    }

    //ROS_INFO("laser_angle_num : %d" , angle_num);
    ROS_INFO("mod_scan_distance : %3.3f" , mod_scan_data.ranges[30]);  //sss
    lineFit(1);
}




float lineFit(int column)
{
	int value_count = 0;

    double A = 0.0 ,B = 0.0 ,C = 0.0,D = 0.0,E = 0.0,F = 0.0;
    double a=0, b=0, TEMP = 0;
    double Line_angle=0,Line_dis=0;
	// ros::Rate loop_rate(30);
	// while(ros::ok())
	// {
	// 	count++;p
	// 	if (count >60)
	// 		break;
	// 	ros::spinOnce();
	// 	loop_rate.sleep();
	// }

    value_count = 0;
    for (int i =0;i < angle_num;i++ )
    {
        //if ( mod_scan_data.ranges[i] > 0.05)
        {
            position_x = mod_scan_data.ranges[i]*cos(mod_scan_data.angle_min - 0.016);
            position_y = mod_scan_data.ranges[i]*sin(mod_scan_data.angle_min - 0.016);
            // position_x = ranges[i]*cos(angle_increment*(i+table_index) - 0.016);
            // position_y = ranges[i]*sin(angle_increment*(i+table_index) - 0.016);
            //ROS_INFO("index:%d,data:%3.3f,x:%3.3f,y:%3.3f",i+table_index,ranges[i],position_x,position_y);
            A += position_y * position_y;
            B += position_y;
            C += position_y * position_x;
            D += position_x;
            value_count++;
        }
    }

    //执行拟合  数据点position_x[value_count]，position_y[value_count] ，数量value_count
    //以position_y为横轴，position_x为纵轴

    // 计算斜率a和截距b
    TEMP = (value_count*A - B*B);

    ROS_INFO("A:%3.3f , B:%3.3f , C:%3.3f , D:%3.3f , TEMP:%3.3f",A,B,C,D,TEMP);

    if( TEMP>-0.0001 &&  TEMP<0.0001)// 判断分母不为0
    {
        a = 1;
        b = 0;
        ROS_INFO("horison Line : %f",TEMP);
        //break;
    }
    else
    {
        a = (value_count*C - B*D) / TEMP;
        b = (A*D - B*C) / TEMP;
        Line_angle = atan(a)*180/3.1415926 + 0.0;
        Line_dis = sin(atan(a))*b/a;
        ROS_INFO("Line angle : %3.3f , Line_dis : %3.3f " ,Line_angle,Line_dis);
        return Line_angle;
    }

	return 0;
}

double pi=3.1415926;
double Laser_Base_Yaw_bias= -0.0;

void Init_pose_calculation(const sensor_msgs::LaserScan & temp_laser)
{
    static float Left__distance=0.0,Right_distance=0.0,Front_distance=0.0,Temp_angel=0.0;
    static float Left__distance_buf[50],Right_distance_buf[50],Front_distance_buf[50],Temp_angel_buf[50];
           float Left__distance_sum=0.0,Right_distance_sum=0.0,Front_distance_sum=0.0,Temp_angel_sum=0.0;
    static int filter_size=20,filter_count=0;
    static float Left__index=0,Right_index=0,Front_index=0;

    Right_index = (-pi/2 - temp_laser.angle_min - Laser_Base_Yaw_bias)/temp_laser.angle_increment;
    Front_index = ( 0.0  - temp_laser.angle_min - Laser_Base_Yaw_bias)/temp_laser.angle_increment;
    Left__index = ( pi/2 - temp_laser.angle_min - Laser_Base_Yaw_bias)/temp_laser.angle_increment;

    Right_distance_buf[filter_count] = temp_laser.ranges[Right_index];
    Front_distance_buf[filter_count] = temp_laser.ranges[Front_index];
    Left__distance_buf[filter_count] = temp_laser.ranges[Left__index];


    for(int i=0;i<filter_size;i++)
    {
        Right_distance_sum += Right_distance_buf[i];
        Front_distance_sum += Front_distance_buf[i];
        Left__distance_sum += Left__distance_buf[i];
    }

    // Right_distance = temp_laser.ranges[Right_index];
    // Front_distance = temp_laser.ranges[Front_index];
    // Left__distance = temp_laser.ranges[Left__index];
    Right_distance = Right_distance_sum/filter_size;
    Front_distance = Front_distance_sum/filter_size;
    Left__distance = Left__distance_sum/filter_size;
    Temp_angel = acos(5.9/(Right_distance + Left__distance))/pi*180;  //小
    // Temp_angel = acos(14.9/(Right_distance + Left__distance))/pi*180;  //大场地

    Temp_angel_buf[filter_count] = Temp_angel;

    for(int i=0;i<filter_size;i++)
    {
        Temp_angel_sum += Temp_angel_buf[i];
    }
    Temp_angel = Temp_angel_sum/filter_size;

    mod_scan_data = temp_laser;
    // mod_scan_data.intensities[Right_index] = 2000;
    // mod_scan_data.intensities[Front_index] = 2000;
    // mod_scan_data.intensities[Left__index] = 2000;

    //ROS_INFO("angle_min : %3.3f , angle_max : %3.3f , angle_increment :3.5f" ,temp_laser.angle_min,temp_laser.angle_max,temp_laser.angle_increment);
    //ROS_INFO("Left_ind : %3.3f , Front_ind : %3.3f , Right_ind : %3.3f " ,Left__index,Front_index,Right_index);
    ROS_INFO("Left_dis : %3.3f , Front_dis : %3.3f , Right_dis : %3.3f , angel : %3.2f" ,Left__distance,Front_distance,Right_distance,Temp_angel);

    if(filter_count++>=filter_size) filter_count=0;
    receive_flag=1;

}



int main(int argc,char **argv)
{

	ros::init(argc,argv,"LaserScan_LineFit");
	ros::NodeHandle n;


	ros::Subscriber laser_scan_data = n.subscribe("/scan",20,Init_pose_calculation);
    ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("/mod_scan", 20);

	ros::Rate loop_rate(20);
	int count = 0;

    while(ros::ok())
    {
        ros::spinOnce();
        ROS_INFO("wait for /scan topics ...  " );
        loop_rate.sleep();
        if(receive_flag==1) break;
    }

	while(ros::ok())
	{
	    count++;
        //lineFit(1);

		ros::spinOnce();
		loop_rate.sleep();

        if(receive_flag==1)
        {
            receive_flag = 0;
            laser_pub.publish(mod_scan_data);
        }

	}
}
