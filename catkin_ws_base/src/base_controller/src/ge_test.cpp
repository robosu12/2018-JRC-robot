#include <iostream>
#include <iomanip>
#include <fstream>
#include <ros/ros.h>
#include "ros/callback_queue.h"

#include <id_data_msgs/ID_Data.h>
using namespace std;
//globals
int error_no=0;
int row[20]={0},column[20]={0};
int obj_num=0;
int loop_sys_cnt=0;
int obj_cnt=0;

//1,navigation section,id=2
bool nav_msg_rec_flag;     //data[0]=14
bool nav_start_pos_flag;   //data[0]=0
bool nav_column1_pos_flag; //data[0]=1
bool nav_column2_pos_flag; //data[0]=2
bool nav_column3_pos_flag; //data[0]=3
bool nav_release_pos_flag; //data[0]=4
bool nav_finished_flag;    //data[0]=15
bool command_move_finished_flag=false;

//2,kinect scan section,id=3
bool kinect_scan_start_flag;       //data[0]=1
bool kinect_scan_stop_flag;        //data[0]=0
bool kinect_scan_fail_flag;        //data[0]=13
bool kinect_msg_rec_flag;          //data[0]=14
bool kinect_reset_flag=false;
bool kinect_scan_finished_flag;    //data[0]=15

//3,arm control section,id=4
bool arm_start_fetch_flag;     //data[0]=1
bool arm_stop_fetch_flag;      //data[0]=0
bool arm_keep_fetch_flag;      //data[0]=2
bool arm_release_obj_flag;     //data[0]=3
bool arm_msg_rec_flag;         //data[0]=14
bool arm_act_finished_flag;    //data[0]=15

//derfine 
int set_ontime=0;

void notice_data_clear(id_data_msgs::ID_Data *test);

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
    notice_message.id=0;
    for(char i=0;i<8;i++)notice_message.data[i]=0;

    notice_message.id=notice_msg->id;
    for(char i=0;i<8;i++)notice_message.data[i]=notice_msg->data[i];

    notice_pub_sub::notice_display(notice_message,true);

    //1,navigation section
    if(notice_message.id==2 && notice_message.data[0]==14)//msg received flag
    {
        nav_msg_rec_flag=true;
    }
    if(notice_message.id==2 && notice_message.data[0]==15)//nav finished flag
    {
        nav_finished_flag=true;
    }
    if(notice_message.id==2 && notice_message.data[0]==16)//nav finished flag
    {
        set_ontime++;
        ROS_WARN("ON TIME No.%d",set_ontime);
    }
    if(notice_message.id==2 && notice_message.data[0]==8)
    {
        command_move_finished_flag=true;
        ROS_INFO("Received dashgo command move finished flag");
    }

    //2,kinect scan section
    if(notice_message.id==3 && notice_message.data[0]==14)//kinect msg received flag
    {
        kinect_msg_rec_flag=true;
    }
    if(notice_message.id==3 && notice_message.data[0]==15)//kinect scan finished flag
    {
        kinect_scan_finished_flag=true;
    }
    if(notice_message.id==3 && notice_message.data[0]==13)//kinect scan failed
    {
//        id_data_msgs::ID_Data id_data;
//        id_data.id=3;
//        for(int count=0;count<8;count++) id_data.data[count]=0;
//        id_data.data[0]=1;
//        notice_publisher.publish(id_data);

        row[obj_num]=row[obj_cnt-1];
        column[obj_num]=column[obj_cnt-1];
        obj_num++;
        kinect_reset_flag=true;
        ROS_WARN("Received missing current column flag.Save data No.%d,Row:%d,Col%d",obj_cnt,row[obj_cnt-1],column[obj_cnt-1]);
        kinect_scan_finished_flag=true;


    }
    //3,arm control section
    if(notice_message.id==4 && notice_message.data[0]==14)//arm control msg received flag
    {
        arm_msg_rec_flag=true;
    }
    if(notice_message.id==4 && notice_message.data[0]==15)//arm fetch finished flag
    {
        arm_act_finished_flag=true;
    }
}

void notice_pub_sub::notice_sub_spinner(char set)
{
    if(set==1)
        notice_spinner->start();
    if(set==0)
        notice_spinner->stop();
}

//function declaration

int main_MsgConform_ActFinishedWait(id_data_msgs::ID_Data *notice_data_test,bool *msg_rec_flag,bool *finished_flag,notice_pub_sub* notice_test);
void error_deal(int error_nu);

int main(int argc,char **argv)
{
    ros::init(argc,argv,"main_loop");

    //read the position of JOY from the txt file
    /*
    ifstream Object_position_file("/home/jdx/test_row_column.txt");
    string temp;
    if(!Object_position_file.is_open())
    {
        ROS_ERROR("Failed to open JOY position file");
    }



    while(getline(Object_position_file,temp))
    {
        sscanf(temp.c_str(),"%d,%d",&row[obj_num],&column[obj_num]);
        obj_num++;
    }
    */
    obj_num=1;
    row[0]=1;
    column[0]=2;
    cout<<"Object JOY number:"<<obj_num<<"\t"<<"JOY Object position:"<<endl;
    
    for(int i=0;i<obj_num;i++)
        cout<<"Column:"<<column[i]<<" "<<"Row:"<<row[i]<<endl;
    //Object_position_file.close();

    //ROS Topic publish and event deal...
    notice_pub_sub notice_test;
    int loop_hz=10;
    ros::Rate loop_rate(loop_hz);
    id_data_msgs::ID_Data notice_data;
    notice_data.id=0;
    for(char i=0;i<8;i++) notice_data.data[i]=0;

    int system_count=0;
    while(ros::ok())
    {
        for(loop_sys_cnt=0;loop_sys_cnt<obj_num;loop_sys_cnt++)
        {
            printf("System run No.%d.\n",system_count++);
            obj_cnt++;

            //1,inform dashgo move to the column of object JOY position.
            ROS_INFO("1,inform dashgo move to the column of object JOY position.");
            notice_data_clear(&notice_data);
            notice_data.id=2;
            notice_data.data[0]=column[loop_sys_cnt];
            notice_data.data[1]=row[loop_sys_cnt];
            error_no=main_MsgConform_ActFinishedWait(&notice_data,&nav_msg_rec_flag,&nav_finished_flag,&notice_test);
            error_deal(error_no);

            //2,informs kinect to scan grid shelves
            //sleep(3);
            ROS_INFO("2,informs kinect to scan grid shelves");
            notice_data_clear(&notice_data);
            notice_data.id=3;
            notice_data.data[0]=1;
            notice_data.data[1]=row[loop_sys_cnt];
            notice_data.data[2]=column[loop_sys_cnt];
            error_no=main_MsgConform_ActFinishedWait(&notice_data,&kinect_msg_rec_flag,&kinect_scan_finished_flag,&notice_test);
            error_deal(error_no);
            if(kinect_reset_flag)
            {
                id_data_msgs::ID_Data back_move;
                notice_data_clear(&back_move);
                back_move.id=2;
                back_move.data[0]=6;
                back_move.data[1]=50;
                notice_test.notice_pub_sub_pulisher(back_move);
                while(ros::ok())
                {
                    notice_test.notice_sub_spinner(1);
                    if(command_move_finished_flag)
                    {
                        command_move_finished_flag=false;
                        ROS_INFO("Dashgo achieved move command!");
                        break;
                    }
                }

                kinect_reset_flag=false;
                continue;
            }
            //3,informs kinova arm to fetch object
            ROS_INFO("3.1,informs kinova arm to fetch object");
            notice_data_clear(&notice_data);
            notice_data.id=4;
            notice_data.data[0]=1;
            error_no=main_MsgConform_ActFinishedWait(&notice_data,&arm_msg_rec_flag,&arm_act_finished_flag,&notice_test);
            error_deal(error_no);
            //arm keep pose
            ROS_INFO("3.2,Kinova Start being into Keep Pose ... ");
            notice_data_clear(&notice_data);
            notice_data.id=4;
            notice_data.data[0]=2;
            error_no=main_MsgConform_ActFinishedWait(&notice_data,&arm_msg_rec_flag,&arm_act_finished_flag,&notice_test);
            error_deal(error_no);

            //4,inform dashgo move to the object release position.
            ROS_INFO("4,inform dashgo move to the object release position.");
            notice_data_clear(&notice_data);
            notice_data.id=2;
            notice_data.data[0]=4;
            error_no=main_MsgConform_ActFinishedWait(&notice_data,&nav_msg_rec_flag,&nav_finished_flag,&notice_test);
            error_deal(error_no);
            
			while(set_ontime<2){};
			set_ontime=0;
            //5,inform kinova arm to release the object.
            ROS_INFO("5,inform kinova arm to release the object.");
            notice_data_clear(&notice_data);
            notice_data.id=4;
            notice_data.data[0]=3;
            error_no=main_MsgConform_ActFinishedWait(&notice_data,&arm_msg_rec_flag,&arm_act_finished_flag,&notice_test);
            error_deal(error_no);
			
            //6,inform dashgo move to the task start position.
            ROS_INFO("6,inform dashgo move to the task start position.");
            notice_data_clear(&notice_data);
            notice_data.id=2;
            notice_data.data[0]=0;
            error_no=main_MsgConform_ActFinishedWait(&notice_data,&nav_msg_rec_flag,&nav_finished_flag,&notice_test);
            error_deal(error_no);

            if(0==column[loop_sys_cnt] && 0==row[loop_sys_cnt])
            {
                ROS_INFO("Grasp tasks FINISHED!\n ");
                break;
            }
        }
        return 0;

    }

    return 0;
}

void notice_data_clear(id_data_msgs::ID_Data *test)
{
    test->id=0;
    for(int i=0;i<8;i++) test->data[i]=0;
}

int main_MsgConform_ActFinishedWait(id_data_msgs::ID_Data *notice_data_test,bool *msg_rec_flag,bool *finished_flag,notice_pub_sub* notice_test)
{
    id_data_msgs::ID_Data notice_data;
    int loop_hz=10;
    ros::Rate loop_rate(loop_hz);

    notice_data_clear(&notice_data);
    notice_data.id=notice_data_test->id;
    for(int i=0;i<8;i++) notice_data.data[i]=notice_data_test->data[i];
    notice_test->notice_pub_sub_pulisher(notice_data);
    //data receive judge
    int wait_count=0;
    while(ros::ok())
    {

        if(*msg_rec_flag==true)
        {
            *msg_rec_flag=false;
            break;
        }

        wait_count++;
        if(wait_count%10==0) //send msg again after waiting 1s
        {
            switch (notice_data.id)
            {
                case 2:ROS_ERROR("Dashgo didn't receive msg,Retrying...");break;
                case 3:ROS_ERROR("Kinect didn't receive msg,Retrying...");break;
                case 4:ROS_ERROR("Kinova arm didn't receive msg,Retrying...");break;
                default:break;
            }

            notice_test->notice_pub_sub_pulisher(notice_data);
        }
        if(wait_count>=100)
        {
            error_no=notice_data.id;
            wait_count=0;
            goto next;
        }
        notice_test->notice_sub_spinner(1);
        loop_rate.sleep();
    }
    //navigation action finish judge
    while(ros::ok())
    {
        if(*finished_flag==true)
        {
            *finished_flag=false;
            break;
        }

        notice_test->notice_sub_spinner(1);
        loop_rate.sleep();
    }

next:
    return error_no;
}

void error_deal(int error_nu)
{
    switch (error_nu)
    {
        case 2:
        {
            ROS_ERROR("Dashgo doesn't work normally!");
            break;
        }
        case 3:
        {
            ROS_ERROR("Kinect doesn't work normally!");
            break;
        }
        case 4:
        {
            ROS_ERROR("Kinova Arm doesn't work normally!");
            break;
        }
        default:break;
    }

}
