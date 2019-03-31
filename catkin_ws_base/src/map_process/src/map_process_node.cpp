#include "ros/ros.h"
#include "map_process/map_process.h"

using namespace map_process;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MapProcess");

    ros::NodeHandle n("~");

    map_process::Map_Process map_processor(n);


    ros::Rate rate(5.0);

    while(ros::ok())
    {
        map_processor.create_costmap();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
