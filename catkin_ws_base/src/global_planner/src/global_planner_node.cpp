#include "ros/ros.h"
#include "global_planner/global_planner.h"

using namespace global_planner;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GlobalPlanner");

    ros::NodeHandle n("~");

    global_planner::Global_Planner global_plan(n);

    ros::Rate rate(1.0);

    while(ros::ok())
    {
        global_plan.astar_process();
        ros::spinOnce();
        rate.sleep();
    }



    return 0;
}
