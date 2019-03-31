#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include "ros/ros.h"
#include "vector"
#include "tf/transform_listener.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "id_data_msgs/ID_Data.h"

using namespace std;

namespace global_planner {

struct Node{
    int nIndexX;
    int nIndexY;
    int makerlab;  ///1:free but unvisit ;2:openlist; 3:closedlist; 4:obstacle; 5:startpoint; 6:endpoint;
    float g_value;
    float f_value;
    float h_value;
    int OpenListIndex;
    int father_IndexX;
    int father_IndexY;
};

class Global_Planner{
public:
    Global_Planner(ros::NodeHandle newnode);
    ~Global_Planner();

    void astar_process();

private:
    void mapCallback(nav_msgs::OccupancyGrid newmap);
    void InitialMap(nav_msgs::OccupancyGrid newmap);

    void goalCallback(geometry_msgs::PointStamped newgoal);

    bool updatestartandend();

    void onestepastarprocess(int nIndexX, int nIndexY);
    void addopenlist(int nIndexX, int nIndexY, int fatherIndexX, int fatherIndexY);
    int addclosedlist();
    void cout_G_value(int nIndexX, int nIndexY);
    void cout_H_value(int nIndexX, int nIndexY);
    void sortopenlist(int nIndexX, int nIndexY);
    bool Isavoidputinopenlist(int nIndexX, int nIndexY);
    float distoftwopoint(int n1IndexX, int n1IndexY, int n2IndexX, int n2IndexY);

    bool getastarpath();





    bool IsInMap(float fX, float fY);
    bool LocatePointInMap(float fX, float fY, int *nIndexX, int *nIndexY);
    bool IsInArray(int nIndexX, int nIndexY);
    bool LocateIndexInArray(int nIndexX, int nIndexY, int *nIndex);
    bool IsObstacle(int nArrayIndex);
    bool IsPointStateFree(float fX, float fY);
    void Array2IndexXY(int nIndex, int *nIndexX, int *nIndexY);
    void IndexXY2Array(int nIndexX, int nIndexY, int *nIndex);   
    geometry_msgs::PoseStamped IndexXY2Pose(int nIndexX, int nIndexY);

    geometry_msgs::Point transformF(geometry_msgs::Point in_point, string tar_frameID, string ori_frameID);


    bool b_IsMapInitial;
    bool b_IsOdomInitial;
    bool b_IsGoalInitial;

    string map_FrameId;
    float f_Mapresolution;
    float n_Mapwidth;
    float n_Mapheight;
    float f_Mapwidth;
    float f_Mapheight;
    float f_MapOrigx;
    float f_MapOrigy;




    // paramters for astar
    geometry_msgs::PointStamped msg_startpoint;
    geometry_msgs::PointStamped msg_endpoint;

    geometry_msgs::Point msg_placementpoint;

    int n_startindex;
    int n_endindex;
    Node* Nodes_;
    vector<int> v_openlist;
    vector<int> v_closedlist;

    nav_msgs::Path msg_pathout;


    nav_msgs::OccupancyGrid msg_Mapin;
    geometry_msgs::PointStamped msg_robot;
    geometry_msgs::PointStamped msg_goal;

    tf::TransformListener listener_;


    ros::Subscriber sub_Map;
    ros::Subscriber sub_Goal;
    ros::Publisher pub_path;
    ros::Publisher pub_notice;
    ros::NodeHandle RosNode_;
};

}

#endif
