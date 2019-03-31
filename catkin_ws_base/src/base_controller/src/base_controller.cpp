/******************************************************************
基于串口通信的ROS小车基础控制器，功能如下：
1.实现ros控制数据通过固定的格式和串口通信，从而达到控制小车的移动
2.订阅了/cmd_vel主题，只要向该主题发布消息，就能实现对控制小车的移动
3.发布里程计主题/odm

串口通信说明：
1.写入串口
（1）内容：左右轮速度，单位为mm/s
（2）格式：１０字节,[右轮速度４字节][左轮速度４字节][结束符"\r\n"２字节]
2.读取串口
（1）内容：小车x,y坐标，方向角，线速度，角速度，单位依次为：mm,mm,rad,mm/s,rad/s
（2）格式：２１字节，[Ｘ坐标４字节][Ｙ坐标４字节][方向角４字节][线速度４字节][角速度４字节][结束符"\n"１字节]
*******************************************************************/
#include "ros/ros.h"  //ros需要的头文件
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "sensor_msgs/LaserScan.h"
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

/****************************************************************************/
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
/*****************************************************************************/
int main_count_i=0;

char serial_init(void);
char Get_Robot_Data(void);
void Analyze_Robot_Data(void);
void Update_Odom_Topic_Data(void);
void Update_Imu_Topic_Data(void);
int serial_ret;

/*****************************************************************************/
float ratio = 1000.0f ;   //转速转换比例，执行速度调整比例
float D = 0.2680859f ;    //两轮间距，单位是m
float linear_temp=0,angular_temp=0;//暂存的线速度和角速度
/****************************************************/
unsigned char send_data_head0 = 0x53,send_data_head1 = 0x49;  //“SI"字符

unsigned char Send_to_Robot_data[30]={0};   //要发给串口的数据
string rec_buffer;  //串口数据接收变量

uint8_t receive_data[80];                      //定义串口数据接收变量
uint8_t receive_data_flag=0;
int receive_data_flag_count=0;

//发送给下位机的左右轮速度，里程计的坐标和方向
union Send_to_Robot //union的作用为实现char数组和float之间的转换
{
    float float_type;
    unsigned char char_type[4];
}Target_Left_Speed,Target_Right_Speed,Target_Position_x,Target_Position_y,Target_Angle;

//receive robot 的左右轮速度，里程计的坐标和方向
union odomtry //union的作用为实现char数组和float之间的转换
{
    float float_type;
    unsigned char char_type[4];
}right_speed_data,left_speed_data,position_x,position_y,oriention,vel_linear,vel_angular;

union IMU  //
{
    float float_type;
    unsigned char char_type[4];
}ACC_x,ACC_y,ACC_z,GY_x,GY_y,GY_z,QUAT_0,QUAT_1,QUAT_2,QUAT_3;

float z_angle_bias_between_odom_map=0;

/*****************************************************************************/

//static tf::TransformBroadcaster odom_broadcaster;//定义tf对象
geometry_msgs::TransformStamped odom_trans;//创建一个tf发布需要使用的TransformStamped类型消息
geometry_msgs::TransformStamped imu_trans;//创建一个tf发布需要使用的TransformStamped类型消息
geometry_msgs::TransformStamped map_2_odom_trans;//创建一个tf发布需要使用的TransformStamped类型消息
nav_msgs::Odometry odom;//定义里程计对象
sensor_msgs::Imu imu_data;
geometry_msgs::Quaternion odom_quat; //四元数变量
geometry_msgs::Pose Temp_Target_pose;

geometry_msgs::Twist target_robot_vel;
geometry_msgs::Twist plan_robot_vel;

double robot_temp_x,robot_temp_y,robot_temp_yaw;
double robot_delta_dis,robot_delta_theta,robot_vel_x,robot_vel_y,robot_vel_angl;
double robot_delta_x,robot_delta_y,robot_delta_yaw,robot_delta_x_last,robot_delta_y_last,robot_delta_yaw_last;
double cart_pitch,cart_roll,cart_yaw;
char Auto_Navigation_flag=0,Arrive_target_position_flag=0,Arrive_target_temp_flag=0,Arrive_target_position_count=0,Changed_target_position_flag=0;
char notice_position_flag=0,notice_position_flag_last=0;
char robot_direction_flag=0,robot_position_x_flag=0;
char fine_tuning_flag=0,Changed_fine_tuning_flag=0;
double fine_tuning_distance=0;
char key_board_enable_flag=0;

//定义covariance矩阵，作用为解决文职和速度的不同测量的不确定性
float covariance[36] = {0.001,   0,  0, 0,   0,     0,  // covariance on gps_x
                        0,  0.001, 0,   0,   0,     0,  // covariance on gps_y
                        0,  0,    1000, 0,     0,    0,  // covariance on gps_z
                        0,  0,    0,     1000, 0,    0,  // large covariance on rot x
                        0,  0,    0,     0,    1000, 0,  // large covariance on rot y
                        0,  0,    0,     0,    0,     1000};  // large covariance on rot z
float imu_data_orientation_covariance[9] = {0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001};
float imu_data_linear_acceleration_covariance[9] = {-1, 0, 0, 0, 0, 0, 0, 0, 0};
float imu_data_angular_velocity_covariance[9] = {0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001};

char target_position_index=0;
///small//////////////////////    1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21    22    23    24
double robot_target_x[50]=  {0.0, 0.0,  6.55, 6.65, 1.65, 1.65, 1.94, 2.18, 2.36, 2.36, 4.83, 4.83, 5.13, 5.43, 5.43, 5.06, 4.83, 4.61, 4.38, 4.40, 1.86, 1.86, 1.56, 1.26, 1.26, 6.65, 0};
double robot_target_y[50]=  {0.0, 0.0,  0.01, 0.09, 0.0,  0.14, 0.14, 0.14, 0.14, 0.0 , 0.0,  0.34, 0.36, 0.36, 0.23,-0.11,-0.11,-0.11,-0.11, 0.14, 0.14,-0.33,-0.35,-0.35,-0.05, 0.01 ,0};
double robot_target_yaw[50]={0.0, 0.0,  0.0,   90,   0.0, -2.0, -2.0, -2.0, -2.0, -2.0, 0.0, -2.0, -2.0, -2.0, -2.0,  178,  178,   178,  178,  178, 180,  178,  178,  178,  178,  90,   0};
double shell_bias=2.88;
double table_bias=0.58;
double set_point_distance=2.64;


////large///////////////////////   1     2      3      4     5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21    22    23    24   25
//double robot_target_x[50]=  {0.0, 0.0,  12.41, 12.57, 3.57, 3.57, 3.79, 4.01, 4.25, 4.25, 8.62, 8.62, 8.94, 9.26, 9.26, 8.90, 8.69, 8.51, 8.28, 8.28, 3.84, 3.84, 3.54, 3.24, 1.74, 1.74, 0};
//double robot_target_y[50]=  {0.0, 0.0, -0.10, -0.10,  4.78, 5.08, 5.08, 5.08, 5.08, 4.78, 4.78, 5.32, 5.32, 5.32, 5.01, 1.86, 1.86, 1.86, 1.86, 2.16, 2.16, 1.64, 1.64, 1.64, 1.64,-0.10, 0};
//double robot_target_yaw[50]={0.0, 0.0,  0.0,   90,    0.0, -2.0, -2.0, -2.0, -2.0, -0.0,  0.0, -2.0, -2.0, -2.0, -2.0,  178,  178,   178,  178,  180, 178,  178,  178,  180,  178,  178,  0};
//double shell_bias=5.88;
//double table_bias=0.58;
//double set_point_distance=6.31;

double robot_target_x_test[50]=  {0.0, 0.0,  6.53, 6.65, 2.22, 2.52, 4.83, 5.13, 5.43, 5.43, 5.17, 4.87, 4.57, 4.27, 1.86, 1.56, 1.26, 1.26, 6.05};
double robot_target_y_test[50]=  {0.0, 0.0,  0.01, 0.09, 0.11, 0.11, 0.38, 0.38, 0.38, 0.18, 0.01, 0.01, 0.01, 0.01,-0.25,-0.25,-0.25,-0.05, 0.01};
double robot_target_yaw_test[50]={0.0, 0.0,  0.0,   90,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  180,  180,  180,  180,  180,  180,  180,  180,  180}; //


double robot_target_x_test1[50]=  {0.0, 0.0,  1.64, 1.64, 1.94, 2.24, 2.44, 2.44, 4.83, 4.83, 5.13, 5.43, 5.43, 5.15, 4.85, 4.55, 4.35, 4.35, 1.86, 1.86, 1.56, 1.26, 1.26, 6.05, 0};
double robot_target_y_test1[50]=  {0.0, 0.0,  0.0,  0.11, 0.11, 0.11, 0.11, 0.0 , 0.0,  0.38, 0.38, 0.38, 0.18,-0.01,-0.01,-0.01,-0.01, 0.14, 0.14,-0.25,-0.25,-0.25,-0.05, 0.11 ,0};
double robot_target_yaw_test1[50]={0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  180,  180,  180,  180,  180,  180,  180,  180,  180,  180,  90,   0}; //

double table_1_x[10]=  {1.64, 1.94, 2.24, 2.44};
double table_1_y[10]=  {1.64, 1.94, 2.24, 2.44};
double table_1_yaw[10]={1.64, 1.94, 2.24, 2.44};
double table_2_x[10]=  {1.64, 1.94, 2.24, 2.44};
double table_2_y[10]=  {1.64, 1.94, 2.24, 2.44};
double table_2_yaw[10]={1.64, 1.94, 2.24, 2.44};
double table_3_x[10]=  {1.64, 1.94, 2.24, 2.44};
double table_3_y[10]=  {1.64, 1.94, 2.24, 2.44};
double table_3_yaw[10]={1.64, 1.94, 2.24, 2.44};
double table_4_x[10]=  {1.64, 1.94, 2.24, 2.44};
double table_4_y[10]=  {1.64, 1.94, 2.24, 2.44};
double table_4_yaw[10]={1.64, 1.94, 2.24, 2.44};

//double robot_target_x_test[20]=  {0.0, 0.0, 1.5, 1.5, 0.0, 2.05, 2.25, 1.95, 1.80, 1.65, 0.0};
//double robot_target_y_test[20]=  {0.0, 0.0, 0.0,-0.6,-0.6,-0.22,-0.22,-1.02,-1.02,-1.02, 0.0};
//double robot_target_yaw_test[20]={0.0, 0.0, 0.0, 179, 179, 0.0,  0.0,  179,  179,  179,  0.0};

// double robot_target_x_test[10]=  {0.0, 0.0, 1.68, 1.96, 1.81, 0.08, 0};
// double robot_target_y_test[10]=  {0.0, 0.0,-0.22,-0.22,-1.03, -1.01, 0};
// double robot_target_yaw_test[10]={0.0, 0.0, 0.0,  0.0,  179,  179, 0};
id_data_msgs::ID_Data notice_data;

double robot_temp_target_x,robot_temp_target_y,robot_temp_target_yaw;
char time_out_process_flag=0;

ros::Time start_move_time;
float delta_move_time;
char record_time_flag=0;


float scan_front_length = 0.0;
// params for obstacle avoidance
ros::Publisher target_pub;
bool Is_need_obstacle_avoidance=false;
bool Is_near_target = false;

char robot_control_state = 1; // 0: controlled by keyboard; 1: controlled by auto


/***************************** notice **************************************/
/***************************** notice **************************************/
/***************************** notice **************************************/

void notice_data_clear(id_data_msgs::ID_Data *test)
{
    test->id=0;
    for(int i=0;i<8;i++) test->data[i]=0;
}

class notice_pub_sub
{
public:
    boost::function<void (const id_data_msgs::ID_Data::ConstPtr&)> notice_pub_sub_msgCallbackFun;

    notice_pub_sub();
    void notice_pub_sub_pulisher(id_data_msgs::ID_Data id_data);
    void notice_display(id_data_msgs::ID_Data notice_msg,bool set);
    void notice_sub_spinner(char set);

private:
    ros::NodeHandle notice_handle;
    ros::Subscriber notice_subscriber;
    ros::Publisher notice_publisher;
    ros::SubscribeOptions notice_ops;
    ros::AsyncSpinner *notice_spinner;
    ros::CallbackQueue notice_callbackqueue;
    void notice_msgCallback(const id_data_msgs::ID_Data::ConstPtr &notice_msg);
};

notice_pub_sub::notice_pub_sub()
{
    notice_pub_sub_msgCallbackFun=boost::bind(&notice_pub_sub::notice_msgCallback,this,_1);
    notice_ops=ros::SubscribeOptions::create<id_data_msgs::ID_Data>(
                "/notice",
                50,
                notice_pub_sub_msgCallbackFun,
                ros::VoidPtr(),
                &notice_callbackqueue
                );
    notice_subscriber=notice_handle.subscribe(notice_ops);
    notice_spinner=new ros::AsyncSpinner(1,&notice_callbackqueue);

    notice_publisher=notice_handle.advertise<id_data_msgs::ID_Data>("/notice",50);
}

void notice_pub_sub::notice_pub_sub_pulisher(id_data_msgs::ID_Data id_data)
{
    notice_publisher.publish(id_data);
}

void notice_pub_sub::notice_display(id_data_msgs::ID_Data notice_msg,bool set)
{
    if(set)
    {
        printf("REC Notice message,ID: %d,Data: ",notice_msg.id);
        for(char i=0;i<8;i++)
        {
            printf("%d ",notice_msg.data[i]);
            if(i==7) printf("\n");
        }
    }
}

void notice_pub_sub::notice_msgCallback(const id_data_msgs::ID_Data::ConstPtr &notice_msg)
{
    id_data_msgs::ID_Data notice_message;
    notice_data_clear(&notice_message);

    notice_message.id=notice_msg->id;
    for(char i=0;i<8;i++)notice_message.data[i]=notice_msg->data[i];

    // navigation section .v
    if(notice_message.id==2)
    {
        if(notice_message.data[0]==1)
        {
            if(notice_message.data[1]>0)//msg received flag
            {
                Changed_target_position_flag=1;
                Arrive_target_position_flag=0;
                Auto_Navigation_flag=1;
                Arrive_target_temp_flag=0;
                time_out_process_flag=0;

                notice_position_flag_last = notice_position_flag;
                notice_position_flag = notice_message.data[1];
                if(notice_position_flag>40) notice_position_flag=40;
                ROS_INFO("Received notice move to %d target point ! \n",notice_position_flag);

		
                robot_temp_target_x = robot_target_x[notice_position_flag];
                robot_temp_target_y = robot_target_y[notice_position_flag];
                robot_temp_target_yaw = robot_target_yaw[notice_position_flag];

		if(notice_position_flag==3)
		{
			robot_temp_target_x = robot_temp_x;
                	robot_temp_target_y = robot_temp_y;
                	robot_temp_target_yaw = robot_target_yaw[notice_position_flag];
		}

                notice_data_clear(&notice_data);
                notice_data.id=2;
                notice_data.data[0]=14;
                notice_pub_sub_pulisher(notice_data);
                ROS_INFO("send response data to notice...");
                notice_pub_sub::notice_display(notice_data,true);
            }  
        } //  end if(notice_message.data[0]==1)
        else
        if(notice_message.data[0]==2 || notice_message.data[0]==3 || notice_message.data[0]==4 || notice_message.data[0]==5)
        {
            fine_tuning_flag = notice_message.data[0];
            fine_tuning_distance = 1.0*notice_message.data[1]/100.0;
            if(fine_tuning_distance>0.2) fine_tuning_distance=0.2;

            Changed_fine_tuning_flag = 1;
            Arrive_target_position_flag=0;
            Auto_Navigation_flag=1;

            if(fine_tuning_flag==2)
            {
                robot_temp_target_x = robot_temp_x + fine_tuning_distance;
                robot_temp_target_y = robot_temp_y;
                robot_temp_target_yaw = robot_temp_yaw;
            }
            else
            if(fine_tuning_flag==3)
            {
                robot_temp_target_x = robot_temp_x - fine_tuning_distance;
                robot_temp_target_y = robot_temp_y;
                robot_temp_target_yaw = robot_temp_yaw;
            }
            else
            if(fine_tuning_flag==4)
            {
                robot_temp_target_x = robot_temp_x;
                robot_temp_target_y = robot_temp_y + fine_tuning_distance;
                robot_temp_target_yaw = robot_temp_yaw;
            }
            else
            if(fine_tuning_flag==5)
            {
                robot_temp_target_x = robot_temp_x;
                robot_temp_target_y = robot_temp_y - fine_tuning_distance;
                robot_temp_target_yaw = robot_temp_yaw;
            }

            notice_data_clear(&notice_data);
            notice_data.id=2;
            notice_data.data[0]=16;
            notice_pub_sub_pulisher(notice_data);
            ROS_INFO("send responseresponse data to notice...");
            notice_pub_sub::notice_display(notice_data,true);
        } //  end if(notice_message.data[0]==2)
        else if(notice_message.data[0]==6) // time_out_process
        {
            if(notice_message.data[1]==6) // time_out_process
            {
                time_out_process_flag=1;

                Changed_target_position_flag=1;
                Arrive_target_position_flag=0;
                Auto_Navigation_flag=1;

                robot_temp_target_x = robot_temp_x;
                robot_temp_target_y = 0.05;
                robot_temp_target_yaw = robot_temp_yaw;

                notice_data_clear(&notice_data);
                notice_data.id=2;
                notice_data.data[0]=14;
                notice_pub_sub_pulisher(notice_data);
                ROS_INFO("send response data to notice...");
                notice_pub_sub::notice_display(notice_data,true);
            }

        }
        else
        {
            // Changed_target_position_flag=0;
            // Arrive_target_position_flag=0;
            // Auto_Navigation_flag=0;
            // notice_position_flag=0;
            ROS_INFO("Received wrong notice data !!! \n");
            notice_pub_sub::notice_display(notice_data,true);
        }
    }
    else
    {
        ROS_INFO("Received wrong ID notice data !!! \n");
    }
   
}

void notice_pub_sub::notice_sub_spinner(char set)
{
    if(set==1)
        notice_spinner->start();
    if(set==0)
        notice_spinner->stop();
}

/***************************** notice **************************************/
/***************************** notice **************************************/
/***************************** notice **************************************/


/*********************** Serial Port initialize *************************/

serial::Serial ser; //声明串口对象 

/*********** 订阅/cmd_vel主题回调函数 callback *************************************************/

void callback_plan_cmd(const geometry_msgs::Twist & cmd_plan)
{
    if(Is_need_obstacle_avoidance == true && Is_near_target == false)
//    if(Is_need_obstacle_avoidance == true)
    {
        plan_robot_vel.angular.z = cmd_plan.angular.z * 180/3.1416 ;//获取/cmd_vel的角速度,rad/s
        plan_robot_vel.linear.x = cmd_plan.linear.x ;//获取/cmd_vel的线速度.m/s
    }
}

void callback_vel(const geometry_msgs::Twist & cmd_input)//订阅/cmd_vel主题回调函数
{
   



    //Changed_target_position_flag=1;
    

    // if(cmd_input.linear.x>0.2)
    // {       
    //     target_position_index=1;
    //     ROS_INFO("target_position_index : 1  \n");
    // }
    // if(cmd_input.linear.x<-0.2)
    // {       
    //     target_position_index=0;
    //     ROS_INFO("target_position_index : 0  \n");
    // }
    // if(cmd_input.angular.z>0.2)
    // {       
    //     target_position_index=2;
    //     ROS_INFO("target_position_index : 2  \n");
    // }
    // if(cmd_input.angular.z<-0.2)
    // {       
    //     target_position_index=3;
    //     ROS_INFO("target_position_index : 3  \n");
    // }

    // if(fabs(cmd_input.linear.x)<0.1 && fabs(cmd_input.angular.z)<0.1)
    // {
    //     Auto_Navigation_flag=0;
    // }
    // else
    // {
    //     Auto_Navigation_flag=1;
    // }

    //Auto_Navigation_flag=0;
    if(cmd_input.linear.x>0.1 && cmd_input.angular.z>0.1)
    {
        robot_control_state = 1;
        plan_robot_vel.angular.z = 0.0 ;//获取/cmd_vel的角速度,rad/s
        plan_robot_vel.linear.x = 0.0 ;//获取/cmd_vel的线速度.m/s
    }
    else
    {
        robot_control_state = 0;
        plan_robot_vel.angular.z = cmd_input.angular.z * 180/3.1416 ;//获取/cmd_vel的角速度,rad/s
        plan_robot_vel.linear.x = cmd_input.linear.x ;//获取/cmd_vel的线速度.m/s
    }
     

    // //将转换好的小车速度分量为左右轮速度
    // //Target_Left_Speed.float_type = linear_temp - 0.5f*angular_temp*D ;
    // //Target_Right_Speed.float_type = linear_temp + 0.5f*angular_temp*D ;
	// Target_Left_Speed.float_type = linear_temp ;
    // Target_Right_Speed.float_type = angular_temp ;
    // Target_Position_x.float_type = 0;
    // Target_Position_y.float_type = 0;
    // Target_Angle.float_type = 0;

    // //存入数据到要发布的左右轮速度消息
    // //left_speed_data.float_type *=ratio;   //放大１０００倍，mm/s
    // //right_speed_data.float_type *=ratio;  //放大１０００倍，mm/s

    // for(int i=0;i<4;i++)    //将左右轮速度存入数组中发送给串口
    // {
    //     Send_to_Robot_data[i+2] = Target_Left_Speed.char_type[i];
    //     Send_to_Robot_data[i+6] = Target_Right_Speed.char_type[i];
    //     Send_to_Robot_data[i+10] = Target_Position_x.char_type[i];
    //     Send_to_Robot_data[i+14] = Target_Position_y.char_type[i];
    //     Send_to_Robot_data[i+18] = Target_Angle.char_type[i];
    // }

    // //在写入串口的左右轮速度数据后加入 data head
    // Send_to_Robot_data[0]=0x53;
    // Send_to_Robot_data[1]=0x49;
	// Send_to_Robot_data[22]=0x41;

    // //写入数据到串口
    // printf("Writing speed data to robot serial port : " );
    // for(char i=0;i<23;i++)
    // {
    //     printf("%x " ,Send_to_Robot_data[i]);
    // }
    // printf("\n" );
    // ser.write(Send_to_Robot_data,23);   //发送串口数据 
}

/*********** 订阅/Target_Pose 主题回调函数 callback_goal *************************************************/

void callback_goal(const geometry_msgs::PoseStamped & Target_Pose)//订阅/cmd_vel主题回调函数
{
    Temp_Target_pose.position.x = Target_Pose.pose.position.x ;//获取/cmd_vel的角速度,rad/s
    Temp_Target_pose.position.y = Target_Pose.pose.position.y ;//获取/cmd_vel的角速度,rad/s
    Temp_Target_pose.position.z = tf::getYaw(Target_Pose.pose.orientation);

    //将转换好的小车速度分量为左右轮速度
    Target_Left_Speed.float_type = 0 ;
    Target_Right_Speed.float_type = 0 ;
    Target_Position_x.float_type = Temp_Target_pose.position.x;
    Target_Position_y.float_type = Temp_Target_pose.position.y;
    Target_Angle.float_type = 0;

    //存入数据到要发布的左右轮速度消息
    //left_speed_data.float_type *=ratio;   //放大１０００倍，mm/s
    //right_speed_data.float_type *=ratio;  //放大１０００倍，mm/s

    for(int i=0;i<4;i++)    //将左右轮速度存入数组中发送给串口
    {
        Send_to_Robot_data[i+2] = Target_Left_Speed.char_type[i];
        Send_to_Robot_data[i+6] = Target_Right_Speed.char_type[i];
        Send_to_Robot_data[i+10] = Target_Position_x.char_type[i];
        Send_to_Robot_data[i+14] = Target_Position_y.char_type[i];
        Send_to_Robot_data[i+18] = Target_Angle.char_type[i];
    }

    //在写入串口的左右轮速度数据后加入 data head
    Send_to_Robot_data[0]=0x53;
    Send_to_Robot_data[1]=0x4a;
	Send_to_Robot_data[22]=0x41;

    //写入数据到串口
    printf("Writing position data to robot serial port : " );
    for(char i=0;i<23;i++)
    {
        printf("%x " ,Send_to_Robot_data[i]);
    }
    printf("\n" );
    ser.write(Send_to_Robot_data,23);   //发送串口数据 
}

/*********** Get_Robot_Data *************************************************/

char Get_Robot_Data(void)
{
    std_msgs::String result;
    uint8_t temp_rece_data[50],head0=0,head1=0;
    int data_lenth=ser.available();

    if(data_lenth>61)
    {      
        //ROS_INFO_STREAM("Read data from serial port" ); 
        //printf("byte : %d \n",data_lenth);     

        while (data_lenth>0)
        {
            ser.read(&head0,1); //获取串口发送来的数据
            if(head0==0x53) break;
            data_lenth--;
            if(data_lenth<1) 
            {
                ROS_WARN_STREAM("receive wrroy data : head0 ! \n");
                return 0;
            }
        }

        while (data_lenth>0)
        {
            ser.read(&head1,1); //获取串口发送来的数据
            if(head1==0x49) break;
            data_lenth--;
            if(data_lenth<1) 
            {
                ROS_WARN_STREAM("receive wrroy data : head1 ! \n");
                return 0;
            }
        }
        
        receive_data[60] =0;
        serial_ret = ser.read(receive_data,61); //获取串口发送来的数据

        if(serial_ret==61 && receive_data[60]==0x41)
        {
            serial_ret=0;
            //ROS_INFO_STREAM("receive correct data from serial \n");
            receive_data_flag = 1;
        }
        else
        {
            ROS_WARN_STREAM("receive wrroy data : wrroy lenth ! \n");
        }

    } // end if(ser.available())
    
}

/*********** Analyze_Robot_Data *************************************************/


void Analyze_Robot_Data(void)
{
	static char count=0;
    if(receive_data_flag==1)
    {
        for(char i=0;i<4;i++)//提取X，Y坐标，方向，线速度，角速度
        {
            position_x.char_type[i]=receive_data[i];
            position_y.char_type[i]=receive_data[i+4];
            oriention.char_type[i]=receive_data[i+8];
            vel_linear.char_type[i]=receive_data[i+12];
            vel_angular.char_type[i]=receive_data[i+16];

            ACC_x.char_type[i] = receive_data[i+20];
            ACC_y.char_type[i] = receive_data[i+24];
            ACC_z.char_type[i] = receive_data[i+28];
            GY_x.char_type[i] = receive_data[i+32];
            GY_y.char_type[i] = receive_data[i+36];
            GY_z.char_type[i] = receive_data[i+40];
            QUAT_0.char_type[i] = receive_data[i+44];
            QUAT_1.char_type[i] = receive_data[i+48];
            QUAT_2.char_type[i] = receive_data[i+52];
            QUAT_3.char_type[i] = receive_data[i+56];
        }
        //receive_data_flag = 0;

		count++;
		if(count<10)
		{
			z_angle_bias_between_odom_map = oriention.float_type;
		}
		else
		{
			count=11;
		}
    }
}

/*********** Update_Odom_Topic_Data *************************************************/


void Update_Odom_Topic_Data(void)
{
    //将X，Y坐标，线速度缩小1000倍
    position_x.float_type/=1000; // m
    position_y.float_type/=1000; // m
    vel_linear.float_type/=1000; // m/s
    //里程计的偏航角需要转换成四元数才能发布
    odom_quat = tf::createQuaternionMsgFromYaw(oriention.float_type);//将偏航角转换成四元数
    //载入坐标（tf）变换时间戳
    odom_trans.header.stamp = ros::Time::now();
    //发布坐标变换的父子坐标系
    odom_trans.header.frame_id = "odom_base";     
    odom_trans.child_frame_id = "base_robot";       
    //tf位置数据：x,y,z,方向
    odom_trans.transform.translation.x = position_x.float_type;
    odom_trans.transform.translation.y = position_y.float_type;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;    
    //发布tf坐标变化
    //odom_broadcaster.sendTransform(odom_trans);


    //载入里程计时间戳
    odom.header.stamp = ros::Time::now(); 
    //里程计的父子坐标系
    odom.header.frame_id = "odom_base";
    odom.child_frame_id = "base_robot";       
    //里程计位置数据：x,y,z,方向
    odom.pose.pose.position.x = position_x.float_type;     
    odom.pose.pose.position.y = position_y.float_type;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;       
    //载入线速度和角速度
    odom.twist.twist.linear.x = vel_linear.float_type;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = vel_angular.float_type; 
    //载入covariance矩阵
    for(int i = 0; i < 36; i++)
    {
        odom.pose.covariance[i] = covariance[i];
    }          
    //发布里程计
    //odom_pub.publish(odom);
}

/*********** Update_Imu_Topic_Data *************************************************/

void Update_Imu_Topic_Data(void)
{
    //载入坐标（tf）变换时间戳
    imu_trans.header.stamp = ros::Time::now();
    //发布坐标变换的父子坐标系
    imu_trans.header.frame_id = "base_link";     
    imu_trans.child_frame_id = "imu_link";       
    //tf位置数据：x,y,z,方向
    imu_trans.transform.translation.x = 0.5;
    imu_trans.transform.translation.y = 0.0;
    imu_trans.transform.translation.z = 0.0;
    imu_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);       
    //发布tf坐标变化
    //odom_broadcaster.sendTransform(odom_trans);

    
    imu_data.header.stamp = ros::Time::now();
    imu_data.header.frame_id = "imu_link";
    //四元数位姿
    imu_data.orientation.x = QUAT_1.float_type;
    imu_data.orientation.y = QUAT_2.float_type;
    imu_data.orientation.z = QUAT_3.float_type;
    imu_data.orientation.w = QUAT_0.float_type;
    //载入covariance矩阵
    for(int i = 0; i < 9; i++)
    {
        //imu_data.orientation_covariance[i] = imu_data_orientation_covariance[i];
    }     
    //线加速度
    imu_data.linear_acceleration.x = ACC_x.float_type; 
    imu_data.linear_acceleration.y = ACC_y.float_type;
    imu_data.linear_acceleration.z = ACC_z.float_type; 
    //载入covariance矩阵
    for(int i = 0; i < 9; i++)
    {
        //imu_data.linear_acceleration_covariance[i] = imu_data_linear_acceleration_covariance[i];
    }     
    //角速度
    imu_data.angular_velocity.x = GY_x.float_type; 
    imu_data.angular_velocity.y = GY_y.float_type; 
    imu_data.angular_velocity.z = GY_z.float_type;
    //载入covariance矩阵
    for(int i = 0; i < 9; i++)
    {
        //imu_data.angular_velocity_covariance[i] = imu_data_angular_velocity_covariance[i];
    }   
    //IMU_pub.publish(imu_data);

}

/*********** get_robot_pose *************************************************/

void get_robot_pose(void)
{
    // //try
    // {
    //      //listener.waitForTransform("/base_link", "/map",ros::Time(0),ros::Duration(30.0));
    //      robot_pose_listener.lookupTransform("/base_link", "/map",ros::Time(0), robot_pose_tf);
    // }
    // //catch (tf::TransformException ex)
    // {
    //      //ROS_ERROR("%s",ex.what());
    //      //ros::Duration(1.0).sleep();
    // }

    // double pitch,roll,yaw;
    // robot_pose_tf.getBasis().getRPY(roll,pitch,yaw);

    // robot_target_yaw = yaw * 180/3.1415926;
    // robot_target_x = robot_pose_tf.getOrigin().x();
    // robot_target_y = robot_pose_tf.getOrigin().y();

    // target_robot_vel.angular.z = 0.5 * atan2(robot_target_y,robot_target_x);
    // target_robot_vel.linear.x = 0.5 * sqrt(pow(robot_target_x, 2) + pow(robot_target_y, 2));
    //robot_target_vel_pub.publish(target_robot_vel);

    //ROS_INFO("robot_pose : X: %2.3f , Y: %2.3f , yaw: %3.2f",robot_target_x,robot_target_y,robot_target_yaw);
}

/*********** get_robot_pose *************************************************/

/*********** abstacle_avoidance *************************************************/

void abstacle_avoidance(const sensor_msgs::LaserScan & temp_laser)
{
    float point_position_x[1080],point_position_y[1080];
    float point_num=0;

    point_num = (temp_laser.angle_max - temp_laser.angle_min) / temp_laser.angle_increment;

    for(int i=0;i<point_num;i++)
    {
        point_position_x[i] = temp_laser.ranges[i]*cos(temp_laser.angle_min+temp_laser.angle_increment*i);
        point_position_y[i] = temp_laser.ranges[i]*sin(temp_laser.angle_min+temp_laser.angle_increment*i);
    }

    for(int i=0;i<point_num;i++)
    {
        if(point_position_x[i]<0.1)
        {
            if(point_position_y[i]<0.5 && point_position_y[i]>-0.3)
            {
                //target_robot_vel.linear.y = 0;
            }
        }

    }


}

/*********** abstacle_avoidance *************************************************/

/*********** abstacle_avoidance *************************************************/
double left_distance_temp=0.0,font_distance_temp = 0.0,distance_correct=0.0;

void distance_fine_tuning(const sensor_msgs::LaserScan & temp_laser)
{
    // float point_position_x[1080],point_position_y[1080];
    // float point_num=0;

    // point_num = (temp_laser.angle_max - temp_laser.angle_min) / temp_laser.angle_increment;

    // for(int i=0;i<point_num;i++)
    // {
    //     point_position_x[i] = temp_laser.ranges[i]*cos(temp_laser.angle_min+temp_laser.angle_increment*i);
    //     point_position_y[i] = temp_laser.ranges[i]*sin(temp_laser.angle_min+temp_laser.angle_increment*i);
    // }

    left_distance_temp = temp_laser.ranges[727];
    font_distance_temp = temp_laser.ranges[367];
    
}

/*********** abstacle_avoidance *************************************************/



/*********** ROS_base_control_to_robot *************************************************/

void ROS_base_control_to_robot(void)
{
    static double robot_temp_x_buf[50],robot_temp_y_buf[50],robot_temp_yaw_buf[50];
    double robot_temp_x_sum=0,robot_temp_y_sum=0,robot_temp_yaw_sum=0;
    static int filter_count=0,Fliter_size=10;
    static double Position_x_I_out=0.0,Position_y_I_out=0.0;
    static double Position_kp=1.0,Position_ki=0.0,Position_kd=0.0;
    static double Position_PID_out_limit=0.5,Angle_PID_out_limit=20,Position_I_out_limit=0.1;

    if(notice_position_flag==2 || notice_position_flag==3)
    {
        //Position_PID_out_limit = 0.4;
    }
    else
    {
        //Position_PID_out_limit = 0.2;
    }

    robot_temp_x_buf[filter_count] = robot_temp_x;
    robot_temp_y_buf[filter_count] = robot_temp_y;
    robot_temp_yaw_buf[filter_count] = robot_temp_yaw;

    if(filter_count++ >= Fliter_size) filter_count=0;
    for(char i=0;i<Fliter_size;i++)
    {
        robot_temp_x_sum += robot_temp_x_buf[i];
        robot_temp_y_sum += robot_temp_y_buf[i];
        robot_temp_yaw_sum += robot_temp_yaw_buf[i];
    }

    robot_temp_x = robot_temp_x_sum/Fliter_size;
    robot_temp_y = robot_temp_y_sum/Fliter_size;
    //robot_temp_yaw = robot_temp_yaw_sum/Fliter_size;
    robot_temp_x_sum = 0;
    robot_temp_y_sum = 0;
    robot_temp_yaw_sum = 0;

    robot_delta_x_last = robot_delta_x;
    robot_delta_y_last = robot_delta_y;
    robot_delta_yaw_last = robot_delta_yaw;
    robot_delta_x = robot_temp_target_x - robot_temp_x;
    //robot_delta_y = robot_temp_target_y - robot_temp_y;
    robot_delta_y = robot_temp_target_y - robot_temp_y + distance_correct;
    //robot_delta_y =  left_distance_temp -robot_temp_target_y;
    robot_delta_yaw =  robot_temp_target_yaw - robot_temp_yaw;
    
    if(robot_delta_yaw> 180) robot_delta_yaw=robot_delta_yaw-360;
    if(robot_delta_yaw<-180) robot_delta_yaw=robot_delta_yaw+360;

    //if(fabs(robot_delta_yaw)>90) robot_delta_yaw=90;

    robot_delta_dis = sqrt(pow(robot_delta_x, 2) + pow(robot_delta_y, 2));
    robot_delta_theta = (atan2(-robot_delta_y,robot_delta_x)*180/3.1416 - robot_temp_yaw);
    robot_vel_x = robot_delta_dis*cos(robot_delta_theta/180*3.1416);
    robot_vel_y = -robot_delta_dis*sin(robot_delta_theta/180*3.1416);
    
    Position_x_I_out += Position_ki*robot_delta_x;
    Position_y_I_out += Position_ki*robot_delta_y;
    if(Position_x_I_out>Position_I_out_limit) Position_x_I_out = Position_I_out_limit;
    if(Position_y_I_out>Position_I_out_limit) Position_y_I_out = Position_I_out_limit;
    if(Position_x_I_out<-Position_I_out_limit) Position_x_I_out = -Position_I_out_limit;
    if(Position_y_I_out<-Position_I_out_limit) Position_y_I_out = -Position_I_out_limit;

    //target_robot_vel.linear.x = Position_kp * robot_delta_x + Position_x_I_out + Position_kd * (robot_delta_x - robot_delta_x_last);
    //target_robot_vel.linear.y = Position_kp * robot_delta_y + Position_y_I_out + Position_kd * (robot_delta_y - robot_delta_y_last);
    //target_robot_vel.linear.x = Position_kp * robot_delta_x ;
    //target_robot_vel.linear.y = Position_kp * robot_delta_y ;                                                                       
    target_robot_vel.linear.x = 0.8*Position_kp * robot_vel_x ;
    target_robot_vel.linear.y = 0.8*Position_kp * robot_vel_y ;
    target_robot_vel.angular.z = -1.0*Position_kp * robot_delta_yaw;

    if(target_robot_vel.linear.x>Position_PID_out_limit) target_robot_vel.linear.x = Position_PID_out_limit;
    if(target_robot_vel.linear.x<-Position_PID_out_limit) target_robot_vel.linear.x = -Position_PID_out_limit;
    if(target_robot_vel.linear.y>Position_PID_out_limit) target_robot_vel.linear.y = Position_PID_out_limit;
    if(target_robot_vel.linear.y<-Position_PID_out_limit) target_robot_vel.linear.y = -Position_PID_out_limit;
    if(target_robot_vel.angular.z>Angle_PID_out_limit) target_robot_vel.angular.z = Angle_PID_out_limit; // max:180/5==36 du/s
    if(target_robot_vel.angular.z<-Angle_PID_out_limit) target_robot_vel.angular.z = -Angle_PID_out_limit;

    if(fabs(robot_delta_x)<0.01)
    {
        target_robot_vel.linear.x = 0;
    }
    if(fabs(robot_delta_y)<0.01)
    {
        target_robot_vel.linear.y = 0;
    }

    // if(fabs(robot_delta_x)<0.03)
    // {
    //     if(fabs(robot_delta_y)<0.01)
    //     {
    //         target_robot_vel.linear.y = 0;
    //     }
    // }
    // else
    // {
    //     target_robot_vel.linear.y = 0;
    // }
    
    if(fabs(robot_delta_yaw)<0.5)
    {
        target_robot_vel.angular.z = 0; 
    }

    
    // if(fabs(robot_delta_x)<0.03 && Changed_target_position_flag==1)
    // {
    //     robot_position_x_flag=1;
    // }
    // if(fabs(robot_delta_x)>=0.03 && robot_position_x_flag==0)
    // {
    //     target_robot_vel.linear.y = 0;
    // }

    if(fabs(robot_delta_yaw)<1 && Changed_target_position_flag==1)
    {
        robot_direction_flag=1;
    }
    if(fabs(robot_delta_yaw)>=1 && robot_direction_flag==0)
    {
        target_robot_vel.linear.x = 0;
        target_robot_vel.linear.y = 0;
    }

    // if(sqrt(pow(robot_delta_x, 2) + pow(robot_delta_y, 2))<0.03 && Changed_target_position_flag==1)
    // {
    //     robot_direction_flag=1;
    // }
    // if(sqrt(pow(robot_delta_x, 2) + pow(robot_delta_y, 2))>=0.03 && robot_direction_flag==0)
    // {
    //    target_robot_vel.angular.z = 0; 
    // }

    // if(fabs(robot_temp_yaw)>90)
    // {
    //     target_robot_vel.linear.x = -target_robot_vel.linear.x;
    //     target_robot_vel.linear.y = -target_robot_vel.linear.y;
    // }

    if(notice_position_flag==9||notice_position_flag==14||notice_position_flag==19) //table1-3: 15-18
    {
        distance_correct = 0.0;
    }

    if(notice_position_flag<5 || time_out_process_flag==1)
    {
        distance_correct = 0.0;
    }

    if(fabs(robot_delta_x)<0.03 && fabs(robot_delta_y)<0.03 && fabs(robot_delta_yaw)<1.0)
    {

        // if(left_distance_temp>2.0)
        // {
        //     distance_correct = (left_distance_temp - 2.87) * cos(robot_temp_yaw/180*3.14);
        // }
        // else
        // {
        //     distance_correct = (left_distance_temp - 0.55) * cos(robot_temp_yaw/180*3.14);
        // }

        if(Arrive_target_temp_flag==0)
        {
            Arrive_target_temp_flag=1;
            // if(notice_position_flag==5)
            // {
            //     distance_correct = (left_distance_temp - 2.87) ;
            // }
            // if(notice_position_flag=11)
            // {
            //     distance_correct = (left_distance_temp - 0.55) ;
            // }
            // if(notice_position_flag==15)
            // {
            //     distance_correct = -(left_distance_temp - 2.87) ;
            // }
            // if(notice_position_flag==21)
            // {
            //     distance_correct = -(left_distance_temp - 0.55) ;
            // }

            if(notice_position_flag==5) //table1-1: 5-8
            {
                distance_correct = (left_distance_temp - shell_bias) ;
            }

            if(notice_position_flag==11) //table1-2: 11-13
            {
                distance_correct = (left_distance_temp - table_bias) ;
            }

            if(notice_position_flag==15) //table1-3: 15-18
            {
                distance_correct = -(left_distance_temp - shell_bias) ;
            }

            if(notice_position_flag==21) //table1-4: 21-23
            {
                distance_correct = -(left_distance_temp - table_bias) ;
            }

//            if(notice_position_flag==9||notice_position_flag==14||notice_position_flag==19) //table1-3: 15-18
//            {
//                distance_correct = 0.0;
//            }

//            if(notice_position_flag<5 || time_out_process_flag==1)
//            {
//                distance_correct = 0.0;
//            }
        }

        if(Changed_target_position_flag==1 || Changed_fine_tuning_flag==1)
        {
            Arrive_target_position_count++;
        }
    }

	if(notice_position_flag==3&&fabs(robot_delta_yaw)<1.0)
	{
//        robot_temp_target_x = robot_temp_x + (set_point_distance-font_distance_temp);
//        robot_temp_target_y = robot_temp_y;
//        robot_temp_target_yaw = robot_temp_yaw;
        Arrive_target_position_flag=1;
    }                     

    
    if(Arrive_target_position_count>8)
    {
        Arrive_target_position_flag=1;
        ROS_INFO("... Arrived_target_position ...  \n");

        //key_board_enable_flag = 0;
        //plan_robot_vel.linear.x =0;
        //plan_robot_vel.angular.z =0;

        // notice_position_flag++;  //目标点间巡航
        // if(notice_position_flag>5) notice_position_flag=1;

        // robot_temp_target_x = robot_target_x_test[notice_position_flag];
        // robot_temp_target_y = robot_target_y_test[notice_position_flag];
        // robot_temp_target_yaw = robot_target_yaw_test[notice_position_flag];

        // Changed_target_position_flag=1;
        // Arrive_target_position_flag=0;
        // Auto_Navigation_flag=1;
    }

    //if(Auto_Navigation_flag!=1 || Changed_target_position_flag!=1 || Changed_fine_tuning_flag!=1)
    //if(Auto_Navigation_flag!=1 || Arrive_target_position_flag==1 && robot_control_state==1)
    if(Auto_Navigation_flag!=1 || robot_control_state!=1)
    {
        target_robot_vel.linear.x = 0;
        target_robot_vel.linear.y = 0;
        target_robot_vel.angular.z = 0;
    }

    if(Is_need_obstacle_avoidance == true && Is_near_target == false)
//    if(Is_need_obstacle_avoidance == true)
    {
        target_robot_vel.linear.x = plan_robot_vel.linear.x;
        target_robot_vel.angular.z = plan_robot_vel.angular.z;
    }

    ////////////////////////////////////////////////////////////////////
    if(robot_control_state==0)
    {
        target_robot_vel.linear.x = plan_robot_vel.linear.x;
	//target_robot_vel.linear.y = plan_robot_vel.linear.x;
        target_robot_vel.angular.z = plan_robot_vel.angular.z; 
    }

//    target_robot_vel.linear.x = 0.4*(font_distance_temp - set_point_distance);
//    target_robot_vel.linear.y = 0;
//    target_robot_vel.angular.z = 0;


    if(target_robot_vel.linear.x>Position_PID_out_limit) target_robot_vel.linear.x = Position_PID_out_limit;
    if(target_robot_vel.linear.x<-Position_PID_out_limit) target_robot_vel.linear.x = -Position_PID_out_limit;
    if(target_robot_vel.linear.y>Position_PID_out_limit) target_robot_vel.linear.y = Position_PID_out_limit;
    if(target_robot_vel.linear.y<-Position_PID_out_limit) target_robot_vel.linear.y = -Position_PID_out_limit;
    if(target_robot_vel.angular.z>Angle_PID_out_limit) target_robot_vel.angular.z = Angle_PID_out_limit; // max:180/5==36 du/s
    if(target_robot_vel.angular.z<-Angle_PID_out_limit) target_robot_vel.angular.z = -Angle_PID_out_limit;

    //将转换好的小车速度分量为左右轮速度
    //Target_Left_Speed.float_type = linear_temp - 0.5f*angular_temp*D ;
    //Target_Right_Speed.float_type = linear_temp + 0.5f*angular_temp*D ;
	Target_Left_Speed.float_type = target_robot_vel.linear.x ;
    Target_Right_Speed.float_type = -target_robot_vel.linear.y ;
    Target_Position_x.float_type = 0;
    Target_Position_y.float_type = 0;
    Target_Angle.float_type = -target_robot_vel.angular.z /180*3.1416;

    for(int i=0;i<4;i++)    //将左右轮速度存入数组中发送给串口
    {
        Send_to_Robot_data[i+2] = Target_Left_Speed.char_type[i];
        Send_to_Robot_data[i+6] = Target_Right_Speed.char_type[i];
        Send_to_Robot_data[i+10] = Target_Position_x.char_type[i];
        Send_to_Robot_data[i+14] = Target_Position_y.char_type[i];
        Send_to_Robot_data[i+18] = Target_Angle.char_type[i];
    }

    //在写入串口的左右轮速度数据后加入 data head
    Send_to_Robot_data[0]=0x53;
    Send_to_Robot_data[1]=0x49;
	Send_to_Robot_data[22]=0x41;

    //写入数据到串口
    ser.write(Send_to_Robot_data,23);   //发送串口数据 

    // printf("Writing speed data to robot serial port : " );
    // for(char i=0;i<23;i++)
    // {
    //     printf("%x " ,Send_to_Robot_data[i]);
    // }
    // printf("\n" );
}

/*********** ROS_base_control_to_robot *************************************************/

void time_out_process(void)
{

}


void obstacle_avoidance_fun(void)
{
    float dx = robot_temp_x - robot_temp_target_x;
    float dy = robot_temp_y - robot_temp_target_y;
    float to_target_dist = sqrt(dx*dx+dy*dy);



    if(to_target_dist < 1.0)
    {
        Is_near_target = true;
    }
    else
    {
        Is_near_target = false;
    }



    if(Is_need_obstacle_avoidance == true && Is_near_target == false)
    {
        geometry_msgs::PointStamped to_obs_target;
        to_obs_target.header.frame_id = "/map";
        to_obs_target.header.stamp = ros::Time::now();
        to_obs_target.point.x = robot_temp_target_x;
        to_obs_target.point.y = robot_temp_target_y;
        target_pub.publish(to_obs_target);
    }
}

/************************************************************/
/************************ main  *****************************/
/************************************************************/

int main(int argc, char **argv)
{ 
	ROS_INFO("this is base controller node ! \n");
    if(serial_init()!=1) //Serial Port initialize
    {
        ROS_WARN_STREAM("serial initial failed ! \n");
        return 0;
    }

    ros::init(argc, argv, "base_controller");//初始化串口节点
    ros::NodeHandle n;  //定义节点进程句柄

    ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 50, callback_vel); //订阅/cmd_vel主题
    ros::Subscriber laser_scan_sub = n.subscribe("/scan",20, distance_fine_tuning);
    ros::Subscriber Goal_sub = n.subscribe("/move_base_simple/goal", 50, callback_goal); //订阅/cmd_vel主题
    ros::Subscriber Plan_sub = n.subscribe("/command_velocity", 50, callback_plan_cmd); //订阅/cmd_vel主题

    ros::Publisher odom_pub= n.advertise<nav_msgs::Odometry>("/odom_base", 50); //定义要发布/odom主题
    ros::Publisher IMU_pub = n.advertise<sensor_msgs::Imu>("/imu", 50);
    target_pub = n.advertise<geometry_msgs::PointStamped>("/clicked_point", 1);
    //ros::Publisher robot_target_vel_pub = n.advertise<geometry_msgs::Twist>("robot_target_vel", 50);                                           

    //ros::Subscriber notice_sub = n.subscribe("/notice", 100, notice_Callback);
    notice_pub_sub notice_test;

    static tf::TransformBroadcaster odom_broadcaster;//定义tf对象
    //static tf::TransformBroadcaster imu_broadcaster;//定义tf对象
	//static tf::TransformBroadcaster map_2_odom_broadcaster;//定义tf对象

    static tf::TransformListener robot_pose_listener;  //创建一个监听对象
    static tf::StampedTransform robot_pose_tf;
    n.param("/base_controller/Is_need_obstacle_avoidance", Is_need_obstacle_avoidance, bool(false));
    
	//serial_init(); //Serial Port initialize

    ros::Rate loop_rate(20);//设置周期休眠时间

    Changed_target_position_flag=1;
    Arrive_target_position_flag=0;
    Auto_Navigation_flag=1;
    robot_control_state = 1;
 
    while(ros::ok())
    {
        if(main_count_i++%4==0)
        {
            //ROS_INFO("timestamp : %d , robot_base_data_count : %d",main_count_i,receive_data_flag_count);

            //get_robot_pose(); 
            ROS_INFO("dis_correct : %2.2f , dis_temp : %2.2f, left_dis : %2.2f",distance_correct,left_distance_temp,left_distance_temp);
            ROS_INFO("pose_temp : X: %2.3f , Y: %2.3f , yaw: %3.2f",robot_temp_x,robot_temp_y,robot_temp_yaw);
            //ROS_INFO("robot_vel : X: %2.3f , Y: %2.3f , yaw: %3.2f",target_robot_vel.linear.x,target_robot_vel.linear.y,target_robot_vel.angular.z);
            ROS_INFO("pose_delta : X: %2.3f , Y: %2.3f , yaw: %3.2f",robot_delta_x,robot_delta_y,robot_delta_yaw);
            //ROS_INFO("vel : X: %2.3f , Y: %2.3f , yaw: %3.2f",robot_vel_x,robot_vel_y,robot_delta_theta);
            //ROS_INFO("dis : %3.3f",robot_delta_dis); // angular_temp

            //ROS_INFO("plan : linear_x: %2.3f , angular_z: %2.3f ",plan_robot_vel.linear.x,plan_robot_vel.angular.z);
        }
        
        Get_Robot_Data();

        receive_data_flag=1;
        if(receive_data_flag==1)
        {
            Analyze_Robot_Data();

            Update_Odom_Topic_Data();
			//map_2_odom_broadcaster.sendTransform(map_2_odom_trans); ////
            //odom_pub.publish(odom);
            //odom_broadcaster.sendTransform(odom_trans);		

            Update_Imu_Topic_Data();   
            //imu_broadcaster.sendTransform(imu_trans);
			//IMU_pub.publish(imu_data);

            receive_data_flag = 0;
            receive_data_flag_count++;
        }

        odom_pub.publish(odom);
		odom_broadcaster.sendTransform(odom_trans);	
        IMU_pub.publish(imu_data);

        try
        {
            robot_pose_listener.lookupTransform("/map", "/base_link",ros::Time(0), robot_pose_tf);
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR( "%s",ex.what());
            ros::Duration(0.5).sleep();
            //sleep(1);
        }
        
        robot_pose_tf.getBasis().getRPY(cart_roll,cart_pitch,cart_yaw);
        robot_temp_yaw = -cart_yaw*180/3.1416 ; // yaw * 180/3.1415926
        robot_temp_x = robot_pose_tf.getOrigin().x();
        robot_temp_y = robot_pose_tf.getOrigin().y();


        obstacle_avoidance_fun();

//        if(Changed_target_position_flag==1 && record_time_flag==0)
//        {
//            start_move_time = ros::Time::now();
//            record_time_flag=1;
//        }
//        delta_move_time = (ros::Time::now() - start_move_time).toSec();
//        if(delta_move_time>20 && record_time_flag==1)
//        {
//            Arrive_target_position_flag=1;
//            record_time_flag=0;
//            delta_move_time=0;
//        }


        robot_control_state = 1;
        ROS_base_control_to_robot();

        if(Arrive_target_position_flag==1)
        {
            if(Changed_target_position_flag==1)
            {
                notice_data_clear(&notice_data);
                notice_data.id=2;
                notice_data.data[0]=15;
                notice_test.notice_pub_sub_pulisher(notice_data);
                notice_test.notice_display(notice_data,true);

                ROS_INFO("... Arrived_target_position ! \n");
                ROS_INFO("... send arrived information to notice ... \n");
                //Arrive_target_position_flag=0;
            }
            else
            if(Changed_fine_tuning_flag==1)
            {
                notice_data_clear(&notice_data);
                notice_data.id=2;
                notice_data.data[0]=17;
                notice_test.notice_pub_sub_pulisher(notice_data);
                notice_test.notice_display(notice_data,true);

                ROS_INFO("... Arrived_target_position ! \n");
                ROS_INFO("... send arrived information to notice ... \n");
                //Arrive_target_position_flag=0;
            }
//            else
//            if(time_out_process_flag==1)
//            {
//                notice_data_clear(&notice_data);
//                notice_data.id=2;
//                notice_data.data[0]=66;
//                notice_test.notice_pub_sub_pulisher(notice_data);
//                notice_test.notice_display(notice_data,true);

//                ROS_INFO("... Arrived_time_out_position ! \n");
//                ROS_INFO("... send arrived information to notice ... \n");
//            }
            
            Arrive_target_position_flag=0;
            Changed_target_position_flag=0;
            Changed_fine_tuning_flag=0;
            Arrive_target_position_count=0;
            robot_direction_flag=0;
            Auto_Navigation_flag=0;
            record_time_flag=0;
            
        //    notice_position_flag++;  //目标点间巡航
        //    if(notice_position_flag>23) notice_position_flag=1;

        //    robot_temp_target_x = robot_target_x_test1[notice_position_flag];
        //    robot_temp_target_y = robot_target_y_test1[notice_position_flag];
        //    robot_temp_target_yaw = robot_target_yaw_test1[notice_position_flag];

        //    Changed_target_position_flag=1;
        //    Arrive_target_position_flag=0;
        //    Auto_Navigation_flag=1;
        //    Arrive_target_temp_flag=0;
        }

        // if(Changed_target_position_flag==1)
        // {
        //     robot_temp_target_x = robot_target_x[notice_position_flag];
        //     robot_temp_target_y = robot_target_y[notice_position_flag];
        //     robot_temp_target_yaw = robot_target_yaw[notice_position_flag];
        // }

        //robot_target_vel_pub.publish(target_robot_vel);
        
		//处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce();//周期执行
        //ROS_INFO("... test111 ! \n");
        loop_rate.sleep();//周期休眠
        //usleep(10000);
        //ROS_INFO("... test222 ! \n");

        //程序周期性调用
        //ros::spinOnce();  //callback函数必须处理所有问题时，才可以用到

        notice_test.notice_sub_spinner(1);
        
    }

    return 0;

}

char serial_init(void)
{
	try 
    { 
    //设置串口属性，并打开串口 
        ser.setPort("/dev/serial_base"); 
        ser.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    } 
  
    //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized successful");
		ROS_INFO_STREAM("Serial Port : /dev/serial_base , 115200"); 
        return 1;
    } 
    else 
    { 
        return -1; 
    } 


}




