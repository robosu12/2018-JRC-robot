#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include "ros/ros.h"
#include "vector"
#include "algorithm"
#include "tf/transform_listener.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "id_data_msgs/ID_Data.h"

using namespace std;

namespace local_planner {

struct Parameter
{
    bool start_running;
    float wp_dist;
    float robotR;
    float wt;
    float ws;
    float wc;
    float max_v;
    float sig_s;
    float ds;
};


class Local_Planner{
public:
    Local_Planner(ros::NodeHandle newnode);
    ~Local_Planner();

    void localplan_core();
    void cfgCallback(Parameter &param);

private:
    void scanCallback(sensor_msgs::LaserScan newscan);
    void InitialScan(sensor_msgs::LaserScan newscan);

    void goalCallback(geometry_msgs::PointStamped newgoal); // frame_id /map
    void astarpathCallback(nav_msgs::Path newpath);


    bool getcwp(); // get current waypoint

    bool checkfreeclg(geometry_msgs::Point clg, float *min_dist); // detect the collision candidate local target;

    bool checkonepointfree(float fx, float fy, float *min_dist);

    bool checkreachgoal();

    geometry_msgs::Point transformF(geometry_msgs::Point in_point, string tar_frameID, string ori_frameID);

    void candidatepathspublish();


    bool b_IsScanInitial;
    bool b_IsOdomInitial;
    bool b_IsGoalInitial;
    bool b_IsPathInitial;



    // parameters for scan

    string scan_FrameId;

    float f_anglemin;
    float f_anglemax;
    float f_angle_increment;
    float f_rangemin;
    float f_rangemax;
    vector<float> v_thetas;

    Parameter params_;

    geometry_msgs::PointStamped msg_cwp; // current waypoint


    vector<geometry_msgs::Point> v_waypoints;
    vector<geometry_msgs::Point> v_obstacls;


    geometry_msgs::PointStamped msg_robot;

    geometry_msgs::PointStamped msg_goal;
    geometry_msgs::PointStamped msg_goal_laser_link;
    vector<geometry_msgs::Point> v_candidate_lg;

    geometry_msgs::PointStamped msg_lg_last;

    geometry_msgs::PointStamped msg_lg;
    sensor_msgs::LaserScan msg_Scanin;

    tf::TransformListener listener_;


    ros::Subscriber sub_Scan;
    ros::Subscriber sub_Goal;
    ros::Subscriber sub_Astarpath;
    ros::Publisher pub_localtarget;
    ros::Publisher pub_cmd;
    ros::Publisher pub_candidatepath;
    ros::Publisher pub_notice;
    ros::NodeHandle RosNode_;
};

}

#endif
