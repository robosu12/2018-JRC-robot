
#include "ros/ros.h"  //ros需要的头文件
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
//以下为串口通讯需要的头文件
#include <string>        
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>

#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include <id_data_msgs/ID_Data.h>

/************************************************************/
/************************ main  *****************************/
/************************************************************/
int count;
int main(int argc, char **argv)
{ 
	ROS_INFO("this is comtesr node ! \n");

    ros::init(argc, argv, "test");//初始化串口节点
    ros::NodeHandle n;  //定义节点进程句柄

    ros::Rate loop_rate(50);//设置周期休眠时间

    while(ros::ok())
    {
        ROS_INFO("temp_count : %d ! \n",count++);
		//处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce();//周期执行
        //ROS_INFO("... test111 ! \n");
        loop_rate.sleep();//周期休眠
        //usleep(10000);
        //ROS_INFO("... test222 ! \n");
    }

    return 0;

}

