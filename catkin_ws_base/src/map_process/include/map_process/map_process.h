#ifndef MAP_PROCESS_H
#define MAP_PROCESS_H

#include "ros/ros.h"
#include "vector"
#include "geometry_msgs/PointStamped.h"
#include "tf/transform_listener.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/GetMap.h"

using namespace std;

namespace map_process {

class Map_Process{
public:
    Map_Process(ros::NodeHandle newnode);
    ~Map_Process();

    void create_costmap();

private:
    void scanCallback(sensor_msgs::LaserScan newscan);
    void mapCallback(nav_msgs::OccupancyGrid newmap);
    void InitialMap();
    void InitialScan(sensor_msgs::LaserScan newscan);

    geometry_msgs::Point fromlasertomap(geometry_msgs::Point laser_point);



    bool IsInMap(float fX, float fY);
    bool LocatePointInMap(float fX, float fY, int *nIndexX, int *nIndexY);
    bool IsInArray(int nIndexX, int nIndexY);
    bool LocateIndexInArray(int nIndexX, int nIndexY, int *nIndex);
    bool IsObstacle(int nArrayIndex);
    bool IsPointStateFree(float fX, float fY);


    // parameters for map
    bool b_IsMapInitial;

    string map_FrameId;


    float f_Mapresolution;
    float n_Mapwidth;
    float n_Mapheight;
    float f_Mapwidth;
    float f_Mapheight;
    float f_MapOrigx;
    float f_MapOrigy;


    float f_robotradius;
    int n_diasize;
    int n_obs_threshold;

    // parameters for scan
    bool b_IsScanInitial;

    string scan_FrameId;

    float f_anglemin;
    float f_anglemax;
    float f_angle_increment;
    float f_rangemin;
    float f_rangemax;
    vector<float> v_thetas;

    geometry_msgs::PoseStamped msg_scanpose;


    tf::TransformListener listener_;

    nav_msgs::OccupancyGrid msg_Mapin;
    nav_msgs::OccupancyGrid msg_Mapout;
    sensor_msgs::LaserScan msg_Scan;


    ros::Publisher pub_Map;
    ros::Subscriber sub_Scan;
    ros::Subscriber sub_map;
    ros::ServiceClient sub_globalmap_srv;
    ros::NodeHandle RosNode_;
};

}

#endif
