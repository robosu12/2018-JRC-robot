#include "global_planner/global_planner.h"

namespace global_planner {
Global_Planner::Global_Planner(ros::NodeHandle newnode)
{
    RosNode_ = newnode;
    b_IsMapInitial = false;
    b_IsOdomInitial = false;
    b_IsGoalInitial = false;
    msg_goal.point.x = 0.0;
    msg_goal.point.y = 0.0;

    msg_placementpoint.x = 12.4;
    msg_placementpoint.y = -0.10;

    sub_Map = RosNode_.subscribe("/costmap",1,&Global_Planner::mapCallback,this);
    sub_Goal = RosNode_.subscribe("/clicked_point", 1, &Global_Planner::goalCallback, this);

    pub_path = RosNode_.advertise<nav_msgs::Path>("/Astar_path", 1);
    pub_notice = RosNode_.advertise<id_data_msgs::ID_Data>("/notice", 1);

}



Global_Planner::~Global_Planner(){}



void Global_Planner::mapCallback(nav_msgs::OccupancyGrid newmap)
{
    if(b_IsMapInitial == false)
    {
        InitialMap(newmap);
        b_IsMapInitial = true;
        cout<<"global map is initialed!!!"<<endl;
    }
    msg_Mapin = newmap;
}



void Global_Planner::goalCallback(geometry_msgs::PointStamped newpoint)
{
    if(b_IsGoalInitial == false)
    {
        b_IsGoalInitial = true;
        cout<<"goal is initialed!!!"<<endl;
    }
    msg_goal = newpoint;
}


bool Global_Planner::updatestartandend()
{
    int nIndexX;
    int nIndexY;
    v_openlist.clear();
    v_closedlist.clear();
    msg_pathout.poses.clear();
    Nodes_ = new Node[int(n_Mapwidth)*int(n_Mapheight)];
    for(int i=0;i<n_Mapwidth*n_Mapheight;i++)
    {
        Array2IndexXY(i, &nIndexX, &nIndexY);
        Nodes_[i].nIndexX = nIndexX;
        Nodes_[i].nIndexY = nIndexY;
        if(msg_Mapin.data[i] == 100) //obstacle
        {
            Nodes_[i].makerlab = 4;
        }
        else
        {
            Nodes_[i].makerlab = 1;
        }

        Nodes_[i].g_value = 100000.0;
        Nodes_[i].f_value = 0.0;
        Nodes_[i].h_value = 0.0;
        Nodes_[i].OpenListIndex = -1;
        Nodes_[i].father_IndexX = -1;
        Nodes_[i].father_IndexY = -1;
    }

    int IndexX;
    int IndexY;
    int Index;


    float dx = msg_robot.point.x - msg_goal.point.x;
    float dy = msg_robot.point.y - msg_goal.point.y;
    float head2goal = atan2(dy, dx);
    bool is_robot_in_obstacle = true;
    for(int i=0;i<10;i++)
    {
        float fx = msg_robot.point.x + 0.1*i*cos(head2goal);
        float fy = msg_robot.point.y + 0.1*i*sin(head2goal);
        if(LocatePointInMap(fx-f_MapOrigx, fy-f_MapOrigy, &IndexX, &IndexY))
        {
            LocateIndexInArray(IndexX, IndexY, &Index);
            Nodes_[Index].makerlab = 5;
            Nodes_[Index].g_value = 0;
            v_openlist.push_back(Index);
            n_startindex = Index;
            Nodes_[Index].OpenListIndex = v_openlist.size()-1;
            is_robot_in_obstacle = false;
            break;
        }
    }

    if(is_robot_in_obstacle)
    {
        cout<<msg_robot.point.x<<","<<msg_robot.point.y<<endl;
        cout<<f_MapOrigx<<","<<f_MapOrigy<<endl;
        cout<<msg_robot.point.x-f_MapOrigx<<","<<msg_robot.point.y-f_MapOrigy<<endl;
        ROS_ERROR("robot is in obstacle, the wrong may be caused by tf error!!!");
        return false;
    }


    dx = msg_goal.point.x - msg_placementpoint.x;
    dy = msg_goal.point.y - msg_placementpoint.y;
    if(sqrt(dx*dx+dy*dy) < 1.5)
    {
        msg_goal.point.x = 12.0;
    }


//    if(LocatePointInMap(msg_robot.point.x-f_MapOrigx, msg_robot.point.y-f_MapOrigy, &IndexX, &IndexY))
//    {
//        LocateIndexInArray(IndexX, IndexY, &Index);
//        Nodes_[Index].makerlab = 5;
//        Nodes_[Index].g_value = 0;
//        v_openlist.push_back(Index);
//        n_startindex = Index;
//        Nodes_[Index].OpenListIndex = v_openlist.size()-1;
//    }
//    else
//    {

//        cout<<msg_robot.point.x<<","<<msg_robot.point.y<<endl;
//        cout<<f_MapOrigx<<","<<f_MapOrigy<<endl;
//        cout<<msg_robot.point.x-f_MapOrigx<<","<<msg_robot.point.y-f_MapOrigy<<endl;
//        ROS_ERROR("robot is in obstacle, the wrong may be caused by tf error!!!");
//        return false;
//    }

    if(IsPointStateFree(msg_goal.point.x-f_MapOrigx, msg_goal.point.y-f_MapOrigy))
    {
        LocatePointInMap(msg_goal.point.x-f_MapOrigx, msg_goal.point.y-f_MapOrigy, &IndexX, &IndexY);
        LocateIndexInArray(IndexX, IndexY, &Index);
        Nodes_[Index].makerlab = 6;
        n_endindex = Index;
    }
    else
    {
        id_data_msgs::ID_Data notice_data;
        notice_data.id=2;
        notice_data.data[0]=15;
        pub_notice.publish(notice_data);
        ROS_ERROR("target is unreachable!!! \n");
        ROS_ERROR("... send change target information to notice ... \n");
        ROS_ERROR("the goal is in obstacle, which is unreachable!!!");
        return false;
    }
    return true;
}

void Global_Planner::astar_process()
{
    geometry_msgs::Point base_point;
    base_point.x = 0.0;
    base_point.y = 0.0;

    //transform from laser_link to map
    msg_robot.point = transformF(base_point, "/map", "laser_link");
//    msg_robot.point = transformF(base_point, "/map", "hokuyo_link");



    if(b_IsMapInitial == false)
        return;

    if(updatestartandend() == false)
    {
        msg_pathout.poses.clear();
        msg_pathout.header.frame_id = "/map";
        msg_pathout.header.stamp = ros::Time::now();
        pub_path.publish(msg_pathout);
        cout<<"No path is found!!!"<<endl;
        return;
    }

    int nIndexX;
    int nIndexY;
    int nIndex;
    while(true)
    {
        nIndex = addclosedlist();
        if(nIndex == -1)
        {
            break;
        }
        Array2IndexXY(nIndex,&nIndexX,&nIndexY);
        //ROS_INFO("nIndex: %d,nIndexX: %d,nIndexY: %d",nIndex,nIndexX,nIndexY);
        onestepastarprocess(nIndexX, nIndexY);
    }

    if(getastarpath())
    {
        msg_pathout.header.frame_id = "/map";
        msg_pathout.header.stamp = ros::Time::now();
        pub_path.publish(msg_pathout);
        cout<<"obtain a success A star path!!!"<<endl;
    }

}

bool Global_Planner::getastarpath()
{
    int nIndex;
    int nIndexX;
    int nIndexY;
    int fatherIndex;
    int fatherIndexX;
    int fatherIndexY;
    geometry_msgs::PoseStamped pose;
    nIndex = n_endindex;
    Array2IndexXY(nIndex,&nIndexX,&nIndexY);
    fatherIndexX = Nodes_[nIndex].father_IndexX;
    fatherIndexY = Nodes_[nIndex].father_IndexY;
    IndexXY2Array(fatherIndexX,fatherIndexY,&fatherIndex);
    if(fatherIndexX<0||fatherIndexY<0)
    {
        ROS_INFO("NO path found!!!");
        return false;
    }
    while(true)
    {
        pose = IndexXY2Pose(nIndexX,nIndexY); //elevation_map
        msg_pathout.poses.push_back(pose);
        nIndexX = fatherIndexX;
        nIndexY = fatherIndexY;
        nIndex = fatherIndex;
        fatherIndexX = Nodes_[nIndex].father_IndexX;
        fatherIndexY = Nodes_[nIndex].father_IndexY;
        IndexXY2Array(fatherIndexX,fatherIndexY,&fatherIndex);
        if(nIndex == n_startindex)
        {
            pose = IndexXY2Pose(nIndexX,nIndexY);
            msg_pathout.poses.push_back(pose);
            break;
        }
    }

    if(msg_pathout.poses.size()>6)
    {
        return true;
    }


    return false;
}

void Global_Planner::onestepastarprocess(int nIndexX, int nIndexY)
{
    addopenlist(nIndexX-1,nIndexY-1,nIndexX,nIndexY);  //down right
    addopenlist(nIndexX,nIndexY-1,nIndexX,nIndexY);    //     right
    addopenlist(nIndexX+1,nIndexY-1,nIndexX,nIndexY);  //up   right
    addopenlist(nIndexX-1,nIndexY,nIndexX,nIndexY);    //down
    addopenlist(nIndexX+1,nIndexY,nIndexX,nIndexY);    //up
    addopenlist(nIndexX-1,nIndexY+1,nIndexX,nIndexY);  //down left
    addopenlist(nIndexX,nIndexY+1,nIndexX,nIndexY);    //     left
    addopenlist(nIndexX+1,nIndexY+1,nIndexX,nIndexY);  //up   left
}


void Global_Planner::addopenlist(int nIndexX, int nIndexY, int fatherIndexX, int fatherIndexY)
{
    int nIndex;
    int fatherIndex;
    int OpenlistIndex;
    int temp = -1;
    IndexXY2Array(nIndexX,nIndexY,&nIndex);
    IndexXY2Array(fatherIndexX,fatherIndexY,&fatherIndex);
    if((nIndexX>-1)&&(nIndexY>-1)&&(nIndexX<n_Mapwidth)&&(nIndexY<n_Mapheight))
    {
        if(Nodes_[nIndex].makerlab == 2)
        {
            float new_g_value = distoftwopoint(nIndexX, nIndexY, fatherIndexX, fatherIndexY)+Nodes_[fatherIndex].g_value;
            if(Nodes_[nIndex].g_value>new_g_value)
            {
                Nodes_[nIndex].g_value = new_g_value;
                Nodes_[nIndex].f_value = Nodes_[nIndex].h_value+Nodes_[nIndex].g_value;
                Nodes_[nIndex].father_IndexX = fatherIndexX;
                Nodes_[nIndex].father_IndexY = fatherIndexY;
                OpenlistIndex = Nodes_[nIndex].OpenListIndex;

                while((OpenlistIndex<v_openlist.size()-1)&&(Nodes_[nIndex].f_value<Nodes_[v_openlist[OpenlistIndex+1]].f_value))
                {
                    temp = v_openlist[OpenlistIndex+1];
                    v_openlist[OpenlistIndex+1] = v_openlist[OpenlistIndex];
                    v_openlist[OpenlistIndex] = temp;
                 //   ROS_INFO("_OpenList[OpenlistIndex]: %d,_OpenList[OpenlistIndex+1]: %d,nIndex: %d",_OpenList[OpenlistIndex],_OpenList[OpenlistIndex+1],nIndex);
                    Nodes_[v_openlist[OpenlistIndex]].OpenListIndex = OpenlistIndex;
                    Nodes_[nIndex].OpenListIndex = OpenlistIndex+1;
                    OpenlistIndex++;
                }
            }
        }
        else if(!Isavoidputinopenlist(nIndexX,nIndexY))
        {
            if(Nodes_[nIndex].makerlab != 6)
            {
                Nodes_[nIndex].makerlab = 2;
            }
            Nodes_[nIndex].father_IndexX = fatherIndexX;
            Nodes_[nIndex].father_IndexY = fatherIndexY;
            cout_G_value(nIndexX,nIndexY);
            cout_H_value(nIndexX,nIndexY);
            Nodes_[nIndex].f_value = Nodes_[nIndex].g_value+Nodes_[nIndex].h_value;
            sortopenlist(nIndexX,nIndexY);
        }
    }

}

int Global_Planner::addclosedlist()
{
    int nIndex;
    if(v_openlist.size()>0)
    {
        nIndex = v_openlist.back();
        v_openlist.pop_back();
        if(Nodes_[nIndex].makerlab == 6)
        {
            ROS_INFO("we have find the goal!!!");
            return -1;
        }
        else
        {
            Nodes_[nIndex].makerlab = 3;
            Nodes_[nIndex].OpenListIndex = -1;
            v_closedlist.push_back(nIndex);
            return nIndex;
        }
    }
    else
    {
        ROS_INFO("No right path is in the the target!!!");
        return -1;
    }
}

void Global_Planner::cout_G_value(int nIndexX, int nIndexY)
{
    int nIndex;
    int fatherIndex;
    int fatherIndexX;
    int fatherIndexY;
    float dist2fatherX;
    float dist2fatherY;
    float dist2father;
    IndexXY2Array(nIndexX,nIndexY,&nIndex);
    fatherIndexX = Nodes_[nIndex].father_IndexX;
    fatherIndexY = Nodes_[nIndex].father_IndexY;
    IndexXY2Array(fatherIndexX,fatherIndexY,&fatherIndex);
    dist2fatherX = float(fatherIndexX-nIndexX);
    dist2fatherY = float(fatherIndexY-nIndexY);
    dist2father = sqrt(dist2fatherX*dist2fatherX+dist2fatherY*dist2fatherY);
   // ROS_INFO("nIndexX: %d,nIndexY: %d,dist2father: %f",nIndexX,nIndexY,dist2father);
    Nodes_[nIndex].g_value = Nodes_[fatherIndex].g_value+dist2father;
    //ROS_INFO("GValue: %f", _Nodes[nIndex].g_value);
}

void Global_Planner::cout_H_value(int nIndexX, int nIndexY)
{
    int nIndex;
    float h_value;
    int endIndexX;
    int endIndexY;
    IndexXY2Array(nIndexX,nIndexY,&nIndex);
    Array2IndexXY(n_endindex,&endIndexX,&endIndexY);
    float distX = float(endIndexX-nIndexX);
    float distY = float(endIndexY-nIndexY);
    float dist = sqrt(distX*distX+distY*distY);
    Nodes_[nIndex].h_value = dist;
}


void Global_Planner::sortopenlist(int nIndexX, int nIndexY)
{
    int nIndex;
    int openlistIndex;
    int temp = -1;
    IndexXY2Array(nIndexX,nIndexY,&nIndex);
    v_openlist.push_back(nIndex);
    Nodes_[nIndex].OpenListIndex = v_openlist.size()-1;
    if(v_openlist.size()>1)
    {
         openlistIndex = v_openlist.size()-2;
         while((openlistIndex>-1)&&(Nodes_[nIndex].f_value>Nodes_[v_openlist[openlistIndex]].f_value))
         {
             temp = v_openlist[openlistIndex+1];
             v_openlist[openlistIndex+1] = v_openlist[openlistIndex];
             v_openlist[openlistIndex] = temp;
             Nodes_[v_openlist[openlistIndex+1]].OpenListIndex = openlistIndex+1;
             Nodes_[nIndex].OpenListIndex = openlistIndex;
            // ROS_INFO("_OpenList[OpenlistIndex]: %d,_OpenList[OpenlistIndex+1]: %d",_OpenList[openlistIndex],_OpenList[openlistIndex+1]);
             openlistIndex--;
         }
    }
}


bool Global_Planner::Isavoidputinopenlist(int nIndexX, int nIndexY)
{
    int nIndex;
    IndexXY2Array(nIndexX,nIndexY,&nIndex);
    if((nIndexX<0||nIndexY<0)||(Nodes_[nIndex].makerlab == 4)||(Nodes_[nIndex].makerlab) == 3)
    {
        return true;
    }
    return false;
}

float Global_Planner::distoftwopoint(int n1IndexX, int n1IndexY, int n2IndexX, int n2IndexY)
{
    float distX = float(n2IndexX-n1IndexX);
    float distY = float(n2IndexY-n1IndexY);
    float dist = sqrt(distX*distX+distY*distY)*f_Mapresolution;
    return dist;
}

void Global_Planner::InitialMap(nav_msgs::OccupancyGrid newmap)
{

    map_FrameId = newmap.header.frame_id;



    f_Mapresolution = newmap.info.resolution;
    n_Mapheight = int(newmap.info.height);
    n_Mapwidth = int(newmap.info.width);
    f_Mapheight = n_Mapheight*f_Mapresolution;
    f_Mapwidth = n_Mapwidth*f_Mapresolution;
    f_MapOrigx = newmap.info.origin.position.x;
    f_MapOrigy = newmap.info.origin.position.y;
}





bool Global_Planner::IsInMap(float fX, float fY)
{

    if(fX>f_Mapwidth || fX<0 || fY>f_Mapheight || fY<0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool Global_Planner::LocatePointInMap(float fX, float fY, int *nIndexX, int *nIndexY)
{
    if(!IsInMap(fX, fY))
    {
        return false;
    }
    /** temp variables*/
    int indexX = 0;
    int indexY = 0;

    indexX = ceil(fX/f_Mapresolution);
    indexY = ceil(fY/f_Mapresolution);

    *nIndexX = indexX;
    *nIndexY = indexY;
    return true;

}

bool Global_Planner::IsInArray(int nIndexX, int nIndexY)
{
    if(nIndexX<n_Mapwidth && nIndexY<n_Mapheight)
    {
        return true;
    }
    else
    {
        return false;
    }
}


bool Global_Planner::LocateIndexInArray(int nIndexX, int nIndexY, int *nIndex)
{
    if(!IsInArray(nIndexX, nIndexY))
    {
        return false;
    }

    *nIndex = nIndexY*n_Mapwidth+nIndexX;
    return true;
}


bool Global_Planner::IsObstacle(int nArrayIndex)
{
    if(msg_Mapin.data[nArrayIndex]==100)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Global_Planner::IsPointStateFree(float fX, float fY)
{
    int nIndexX = 0;
    int nIndexY = 0;
    int nIndex = 0;

    if(!LocatePointInMap(fX, fY, &nIndexX, &nIndexY))
    {
        /** get into boundary, return false*/
        return false;
    }
    if(!LocateIndexInArray(nIndexX, nIndexY, &nIndex))
    {
        /** get into boundary, return false*/
        return false;
    }

    /** check if it is obstacle*/
    if(IsObstacle(nIndex))
    {
        return false;
    }
    else
    {
        return true;
    }
}

void Global_Planner::Array2IndexXY(int nIndex, int *nIndexX, int *nIndexY)
{

    *nIndexX = nIndex%int(n_Mapwidth);
    *nIndexY = nIndex/int(n_Mapwidth);
}



void Global_Planner::IndexXY2Array(int nIndexX, int nIndexY, int *nIndex)
{
    *nIndex = nIndexY*n_Mapwidth+nIndexX;
}

geometry_msgs::PoseStamped Global_Planner::IndexXY2Pose(int nIndexX, int nIndexY)
{
    geometry_msgs::PoseStamped pose;
    float fX = float(nIndexX*f_Mapresolution);
    float fY = float(nIndexY*f_Mapresolution);
    pose.pose.position.x = fX+f_MapOrigx;
    pose.pose.position.y = fY+f_MapOrigy;
    return pose;
}

geometry_msgs::Point Global_Planner::transformF(geometry_msgs::Point in_point, string tar_frameID, string ori_frameID)
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


}
