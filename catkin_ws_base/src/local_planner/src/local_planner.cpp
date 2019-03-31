#include "local_planner/local_planner.h"

namespace local_planner {
Local_Planner::Local_Planner(ros::NodeHandle newnode)
{
    RosNode_ = newnode;
    b_IsScanInitial = false;
    b_IsOdomInitial = false;
    b_IsGoalInitial = false;
    b_IsPathInitial = false;
//    msg_goal.point.x = 0.0;
//    msg_goal.point.y = 0.0;
    msg_lg_last.point.z = 0.0;


    sub_Scan = RosNode_.subscribe("/scan",1,&Local_Planner::scanCallback,this);
    sub_Goal = RosNode_.subscribe("/clicked_point", 1, &Local_Planner::goalCallback, this);
    sub_Astarpath = RosNode_.subscribe("/Astar_path", 1, &Local_Planner::astarpathCallback, this);

    pub_localtarget = RosNode_.advertise<geometry_msgs::PointStamped>("/local_target", 1);
//    pub_cmd = RosNode_.advertise<geometry_msgs::Twist>("/command_velocity", 1);
    pub_cmd = RosNode_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    pub_candidatepath = RosNode_.advertise<nav_msgs::Path>("/candidate_paths", 1);


}


Local_Planner::~Local_Planner(){}

void Local_Planner::cfgCallback(Parameter &param)
{
    params_ = param;
}

void Local_Planner::localplan_core()
{
    if(params_.start_running)
    {
        ros::Time scan_stamp = msg_Scanin.header.stamp;
        if(b_IsPathInitial && b_IsScanInitial && b_IsGoalInitial)//if(b_IsOdomInitial && b_IsPathInitial && b_IsScanInitial)
        {
            geometry_msgs::Point base_point;
            base_point.x = 0.0;
            base_point.y = 0.0;

            //transform from scan frame id to /map
            msg_robot.point = transformF(base_point, "/map", scan_FrameId);

            //transform from /map to scan frame id
            msg_goal_laser_link.point = transformF(msg_goal.point, scan_FrameId, "/map");

            if(checkreachgoal())
                return;

            if(getcwp() == false)
                return;

            v_candidate_lg.clear();

            float dx = msg_cwp.point.x;// - msg_robot.point.x;
            float dy = msg_cwp.point.y;// - msg_robot.point.y;

            float head = atan2(dy, dx);
            float dist = sqrt(dx*dx+dy*dy);

            float dangle = M_PI/30.0; // from -pi/6 to pi/6
            vector<float> obs_score;
            obs_score.clear();
            for(int i=0;i<15;i++)
            {
                float min_dist_left;
                geometry_msgs::Point left_clg;
                float left_angle = head-dangle*i;
                left_clg.x = dist*cos(left_angle);
                left_clg.y = dist*sin(left_angle);
    //            left_clg.x = dx+0.05*i*cos(head-(M_PI/2.0));
    //            left_clg.y = dy+0.05*i*sin(head-(M_PI/2.0));
                left_clg.z = atan2(left_clg.y, left_clg.x);

                if(checkfreeclg(left_clg, &min_dist_left))
                {
                    v_candidate_lg.push_back(left_clg);
                    obs_score.push_back(min_dist_left);
                }

                float min_dist_right;
                geometry_msgs::Point right_clg;
                float right_angle = head+dangle*i;
                right_clg.x = dist*cos(right_angle);
                right_clg.y = dist*sin(right_angle);
    //            right_clg.x = dx+0.05*i*cos(head+(M_PI/2.0));
    //            right_clg.y = dy+0.05*i*sin(head+(M_PI/2.0));
                right_clg.z = atan2(right_clg.y, right_clg.x);

                if(checkfreeclg(right_clg, &min_dist_right))
                {
                    v_candidate_lg.push_back(right_clg);
                    obs_score.push_back(min_dist_right);
                }

            }



            if(v_candidate_lg.size() > 0)
            {
                candidatepathspublish();
                vector<float> obs_costs;  // safety criteria
                vector<float> goal_costs; // time cost criteria
                vector<float> pre_costs;  // consistency criteria
                obs_costs.clear();
                goal_costs.clear();
                pre_costs.clear();

                cout<<"--------------------------------------------"<<endl;
                for(int j=0;j<v_candidate_lg.size();j++)
                {

                    //time cost
                    float dx0 = v_candidate_lg[j].x;
                    float dy0 = v_candidate_lg[j].y;
                    float dist2robot = sqrt(dx0*dx0+dy0*dy0);

                    float dx1 = v_candidate_lg[j].x - msg_goal_laser_link.point.x;
                    float dy1 = v_candidate_lg[j].y - msg_goal_laser_link.point.y;
                    float dist2goal = sqrt(dx1*dx1+dy1*dy1);

                    float goal_cost = dist2robot + dist2goal;

                    goal_costs.push_back(goal_cost);


                    // safety criteria
                    float obs_cost = 0.0;
                    if(obs_score[j] < params_.ds)
                    {
                        obs_cost = pow((1.0/obs_score[j]-1.0/params_.ds),2);
                    }

                    obs_costs.push_back(obs_cost);

                    //consistency

                    float pre_cost = fabs(v_candidate_lg[j].z- msg_lg_last.point.z);
                    pre_costs.push_back(pre_cost);

                }

                float max_goal_cost = *max_element(goal_costs.begin(), goal_costs.end());
                float min_goal_cost = *min_element(goal_costs.begin(), goal_costs.end());

                float max_obs_cost = *max_element(obs_costs.begin(), obs_costs.end());
                float min_obs_cost = *min_element(obs_costs.begin(), obs_costs.end());

                float max_pre_cost = *max_element(pre_costs.begin(), pre_costs.end());
                float min_pre_cost = *min_element(pre_costs.begin(), pre_costs.end());


                vector<float> scores_buf;
                scores_buf.clear();

                /// normlized costs
//                vector<float> nobs_costs;  // safety criteria
//                vector<float> ngoal_costs; // time cost criteria
//                vector<float> npre_costs;  // consistency criteria
//                nobs_costs.clear();
//                ngoal_costs.clear();
//                npre_costs.clear();
                for(int j=0;j<v_candidate_lg.size();j++)
                {
                    float goal_cost, obs_cost, pre_cost;

                    // goal
                    if(max_goal_cost == min_goal_cost)
                    {
                        goal_cost = 0.0;
                    }
                    else
                    {
                        goal_cost = (goal_costs[j]-min_goal_cost)/(max_goal_cost-min_goal_cost);
                    }


                    //obstacle
                    if(max_obs_cost == min_obs_cost)
                    {
                        obs_cost = 0.0;
                    }
                    else
                    {
                        obs_cost = (obs_costs[j]-min_obs_cost)/(max_obs_cost-min_obs_cost);
                    }

                    // consistency
                    if(max_pre_cost == min_pre_cost)
                    {
                        pre_cost = 0.0;
                    }
                    else
                    {
                        pre_cost = (pre_costs[j]-min_pre_cost)/(max_pre_cost-min_pre_cost);
                    }

//                    nobs_costs.push_back(obs_cost);
//                    ngoal_costs.push_back(goal_cost);
//                    npre_costs.push_back(pre_cost);

                    float total_cost = params_.wt*goal_cost+params_.ws*obs_cost+params_.wc*pre_cost;
                    scores_buf.push_back(total_cost);

//                    cout<<"index:"<<j<<" goal cost:"<<goal_cost<<" obs cost:"<<obs_cost<<" pre cost:"<<pre_cost<<" total cost:"<<total_cost<<endl;

                }





                vector<float>::iterator min_dist_iter = min_element(scores_buf.begin(), scores_buf.end());
                int min_index = distance(scores_buf.begin(), min_dist_iter);
                msg_lg.header.frame_id = scan_FrameId;
                msg_lg.header.stamp = ros::Time::now();
                msg_lg.point = v_candidate_lg[min_index];

                geometry_msgs::PointStamped current_lg;
                current_lg.header.frame_id = scan_FrameId;
                current_lg.header.stamp = ros::Time::now();
                current_lg.point = msg_lg.point;
                current_lg.point.z = 0.0;
                pub_localtarget.publish(current_lg);   // the z of msg_lg geometry_msgs/PointStamped is remapped to heading of robot

                msg_lg_last.point = msg_lg.point;

                geometry_msgs::Twist cmd;

                /// get angular speed
                cmd.angular.z = v_candidate_lg[min_index].z;

                if(fabs(cmd.angular.z) > 0.9)
                {
                    cmd.angular.z = 0.9*(cmd.angular.z/fabs(cmd.angular.z));
                }

                if(fabs(v_candidate_lg[min_index].z) > (3.0*M_PI/4.0))
                {
                    cmd.angular.z = fabs(cmd.angular.z);

                }

                /// get linear speed
                if(fabs(v_candidate_lg[min_index].z) < (M_PI/18.0))
                {
                    cmd.linear.x = params_.max_v;
                }
                else if(fabs(v_candidate_lg[min_index].z) < (M_PI/6.0))
                {
                    cmd.linear.x = 0.8*params_.max_v;
                }
                else if(fabs(v_candidate_lg[min_index].z) < (M_PI/3.0))
                {
                    cmd.linear.x = 0.6*params_.max_v;
                }
                else if(fabs(v_candidate_lg[min_index].z) < (M_PI/2.0))
                {
                    cmd.linear.x = 0.2*params_.max_v;
                }
                else
                {
                    cmd.linear.x = 0.0;
                }

                float rf = (float(v_candidate_lg.size()))/30.0;

//                cmd.linear.x = rf*cmd.linear.x;

                pub_cmd.publish(cmd);
            }
            else
            {
                geometry_msgs::Twist cmd;
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.9*(msg_lg_last.point.z/fabs(msg_lg_last.point.z));
                pub_cmd.publish(cmd);
                cout<<"there is no useful local target, starting recovery behavior...."<<endl;
            }

        }

    //    cout<<"mapping time cost: "<<(clockEnd-clockBegin)/float(CLOCKS_PER_SEC)<<endl;

    //    cout<<"scan_stam:"<<scan_stamp<<endl;
    //    cout<<"ros time now:"<<ros::Time::now()<<endl;
    }

}


bool Local_Planner::checkreachgoal()
{
    float dx = msg_robot.point.x - msg_goal.point.x;
    float dy = msg_robot.point.y - msg_goal.point.y;

    float dist2goal = sqrt(dx*dx+dy*dy);

    if(dist2goal < 0.3)
    {
        cout<<"reach the goal success!!!"<<endl;

        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        pub_cmd.publish(cmd);
        return true;
    }
    return false;
}


bool Local_Planner::checkfreeclg(geometry_msgs::Point clg, float *min_dist)
{
    float fdist = 0.05;
    *min_dist = 1000.0;
    float dist = sqrt(clg.x*clg.x+clg.y*clg.y);
    int num = int(dist/fdist);
    for(int i=5;i<num;i++)
    {
        float fx = fdist*i*cos(clg.z);
        float fy = fdist*i*sin(clg.z);

        float dist;
        if(checkonepointfree(fx, fy, &dist) == false)
            return false;

        if(*min_dist > dist)
        {
            *min_dist = dist;
        }
    }
    return true;
}

bool Local_Planner::checkonepointfree(float fx, float fy, float *min_dist)
{
    *min_dist = 10000.0;

    for(int i=0;i<v_obstacls.size();i++)
    {
        float dx = v_obstacls[i].x - fx;
        float dy = v_obstacls[i].y - fy;
        float dist = sqrt(dx*dx+dy*dy);
        if(*min_dist > dist)
        {
            *min_dist = dist;
        }

    }

    if(*min_dist < (params_.robotR))
    {
        return false;
    }
    return true;
}



void Local_Planner::scanCallback(sensor_msgs::LaserScan newscan)
{
    if(b_IsScanInitial == false)
    {
        InitialScan(newscan);
        b_IsScanInitial = true;
        cout<<"scan is initialed!!!"<<endl;
    }
    msg_Scanin = newscan;

    v_obstacls.clear();
    int i = 0;
    while(true)
    {
        float theta = f_anglemin + f_angle_increment*10.0*i;
        if(theta >= f_anglemax)
            break;

        if(msg_Scanin.ranges[10*i] < f_rangemax)
        {
            geometry_msgs::Point obs;
            obs.x = msg_Scanin.ranges[10*i]*cos(theta);
            obs.y = msg_Scanin.ranges[10*i]*sin(theta);
            v_obstacls.push_back(obs);
        }
        i++;
    }

}

void Local_Planner::InitialScan(sensor_msgs::LaserScan newscan)
{
    scan_FrameId = newscan.header.frame_id;//newscan.header.frame_id;
    f_anglemin = newscan.angle_min;
    f_anglemax = newscan.angle_max;
    f_angle_increment = newscan.angle_increment;
    f_rangemin = newscan.range_min;
    f_rangemax = newscan.range_max;
}



void Local_Planner::goalCallback(geometry_msgs::PointStamped newpoint)
{
    if(b_IsGoalInitial == false)
    {
        b_IsGoalInitial = true;
        cout<<"goal is initialed!!!"<<endl;
    }
    msg_goal = newpoint;
}

void Local_Planner::astarpathCallback(nav_msgs::Path newpath)
{

    if(b_IsPathInitial == false)
    {
        b_IsPathInitial = true;
        cout<<"A* global path is initialed!!!"<<endl;
    }

    // It is should be noted that the start point is at the end position of newpath. otherwise the goal is at the beginning.

    v_waypoints.clear();

    if(newpath.poses.size()>0)
    {

        float cum_dist = 0.0;
        geometry_msgs::Point point0;
        point0.x = newpath.poses[newpath.poses.size()-1].pose.position.x;
        point0.y = newpath.poses[newpath.poses.size()-1].pose.position.y;
        v_waypoints.push_back(point0);
        geometry_msgs::Point point1;
        while(true)
        {
            geometry_msgs::PoseStamped temp_pose = newpath.poses.back();
            newpath.poses.pop_back();
            point1.x = temp_pose.pose.position.x;
            point1.y = temp_pose.pose.position.y;
            float dx = point1.x - point0.x;
            float dy = point1.y - point0.y;

            cum_dist += sqrt(dx*dx+dy*dy);

            if((cum_dist) > params_.wp_dist)
            {
                v_waypoints.push_back(point0);
                cum_dist = 0.0;
            }

            point0.x = point1.x;
            point0.y = point1.y;

            if(newpath.poses.size() == 0)
            {
                break;
            }
        }

        msg_goal.point = point0;
        v_waypoints.push_back(point0);
    }

}


bool Local_Planner::getcwp()
{
    vector<float> dist_buf;
    dist_buf.clear();

    if(v_waypoints.size() <= 0)
    {
        cout<<"no target useful!!!"<<endl;
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        pub_cmd.publish(cmd);
        return false;
    }

    for(int i=0;i<v_waypoints.size();i++)
    {
        float dx = v_waypoints[i].x - msg_robot.point.x;
        float dy = v_waypoints[i].y - msg_robot.point.y;
        float dist = sqrt(dx*dx+dy*dy);
        dist_buf.push_back(dist);
    }

    vector<float>::iterator min_dist_iter = min_element(dist_buf.begin(), dist_buf.end());
    int min_index = distance(dist_buf.begin(), min_dist_iter);

    if(min_index == (v_waypoints.size()-1))
    {
        min_index = v_waypoints.size() - 1;
    }
    else
    {
        min_index += 1;
    }

    geometry_msgs::Point map_point = v_waypoints[min_index];

    //transform from /map to scan frame id
    msg_cwp.point = transformF(map_point, scan_FrameId, "/map");

    return true;

}



geometry_msgs::Point Local_Planner::transformF(geometry_msgs::Point in_point, string tar_frameID, string ori_frameID)
{
    geometry_msgs::Point out_point;
    tf::StampedTransform orig_pose;

    try{
      listener_.lookupTransform(tar_frameID, ori_frameID,
                               ros::Time(0), orig_pose);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    geometry_msgs::Quaternion q;
    q.x = orig_pose.getRotation().getX();
    q.y = orig_pose.getRotation().getY();
    q.z = orig_pose.getRotation().getZ();
    q.w = orig_pose.getRotation().getW();
    float yaw = tf::getYaw(q);

    float dx = orig_pose.getOrigin().x();
    float dy = orig_pose.getOrigin().y();

    out_point.x = in_point.x * cos(yaw) - in_point.y * sin(yaw) + dx;
    out_point.y = in_point.x * sin(yaw) + in_point.y * cos(yaw) + dy;

    return out_point;

}


void Local_Planner::candidatepathspublish()
{
    nav_msgs::Path canpaths;
    geometry_msgs::PoseStamped pose;
    int i = 0;
    int j = 0;
    float t=0.0;
    for(i=0; i<v_candidate_lg.size(); i++)
    {
        float dist = sqrt(v_candidate_lg[i].x*v_candidate_lg[i].x+v_candidate_lg[i].y*v_candidate_lg[i].y);
        float ddist = dist/20.0;
        for(j=0; j<=20; j++)
        {
            t=j*ddist;
            pose.pose.position.x = t*cos(v_candidate_lg[i].z);
            pose.pose.position.y = t*sin(v_candidate_lg[i].z);
            canpaths.poses.push_back(pose);
        }

        for(j=20; j>=0; j--)
        {
            t=j*ddist;
            pose.pose.position.x = t*cos(v_candidate_lg[i].z);
            pose.pose.position.y = t*sin(v_candidate_lg[i].z);
            canpaths.poses.push_back(pose);
        }

    }

    canpaths.header.frame_id = scan_FrameId;
    canpaths.header.stamp = ros::Time::now();
    pub_candidatepath.publish(canpaths);
}



}
