#include "ros/ros.h"
#include "local_planner/local_planner.h"
#include "dynamic_reconfigure/server.h"
#include "local_planner/local_planConfig.h"


using namespace local_planner;

Parameter param;


void callback(local_planner::local_planConfig &config, uint32_t level)
{
    param.start_running = config.start_running;
    param.wp_dist = config.wp_dist;
    param.robotR = config.robotR;
    param.wt = config.wt;
    param.ws = config.ws;
    param.wc = config.wc;
    param.max_v = config.max_v;
    param.sig_s = config.sig_s;
    param.ds = config.ds;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "LocalPlanner");

    ros::NodeHandle n("~");

    dynamic_reconfigure::Server<local_planner::local_planConfig> server;
    dynamic_reconfigure::Server<local_planner::local_planConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);


    local_planner::Local_Planner local_plan(n);

    ros::Rate rate(10.0);

    while(ros::ok())
    {
        local_plan.cfgCallback(param);
        local_plan.localplan_core();
        ros::spinOnce();
        rate.sleep();
    }




    return 0;
}
