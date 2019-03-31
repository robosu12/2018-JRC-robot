#include "map_process/map_process.h"

namespace map_process {
Map_Process::Map_Process(ros::NodeHandle newnode)
{
    RosNode_ = newnode;
    b_IsMapInitial = false;
    b_IsScanInitial = false;
    f_robotradius = 0.2;
    RosNode_.param("/map_process/obs_threshold", n_obs_threshold, 100);
    cout<<"/map_process/obs_threshold"<<n_obs_threshold<<endl;
    pub_Map = RosNode_.advertise<nav_msgs::OccupancyGrid>("/costmap",1);
    sub_Scan = RosNode_.subscribe("/scan",1,&Map_Process::scanCallback,this);
    sub_globalmap_srv = RosNode_.serviceClient<nav_msgs::GetMap>("/static_map");
    sub_map = RosNode_.subscribe("/map",1,&Map_Process::mapCallback,this);
}


Map_Process::~Map_Process(){}

void Map_Process::create_costmap()
{

//    if(!b_IsMapInitial)
//    {
//        InitialMap();
//    }

    if(b_IsMapInitial && b_IsScanInitial)
    {

        vector<float> ranges = msg_Scan.ranges;
        for(int i=0;i<ranges.size();i++)
        {
            if(ranges[i] >= f_rangemax)
                continue;

            geometry_msgs::Point scan_point;
            scan_point.x = ranges[i]*cos(v_thetas[i]);
            scan_point.y = ranges[i]*sin(v_thetas[i]);

            geometry_msgs::Point map_point;
            map_point = fromlasertomap(scan_point);

            int nIndexX = 0;
            int nIndexY = 0;
            int nIndex = 0;

            if(!LocatePointInMap(map_point.x-f_MapOrigx, map_point.y-f_MapOrigy, &nIndexX, &nIndexY))
                continue;

            if(!LocateIndexInArray(nIndexX, nIndexY, &nIndex))
                continue;

            int start_indexX = nIndexX - n_diasize;
            int end_indexX = nIndexX + n_diasize;

            int start_indexY = nIndexY - n_diasize;
            int end_indexY = nIndexY + n_diasize;

            if(start_indexX < 0) start_indexX = 0;
            if(start_indexY < 0) start_indexY = 0;
            if(end_indexX > n_Mapwidth) end_indexX = n_Mapwidth;
            if(end_indexY > n_Mapheight) end_indexY = n_Mapheight;

            int Index;
            int IndexX;
            int IndexY;

            for(IndexX=start_indexX; IndexX<end_indexX;IndexX++)
            {
                for(IndexY=start_indexY; IndexY<end_indexY;IndexY++)
                {
                    Index = IndexY*n_Mapwidth + IndexX;
                    msg_Mapout.data[Index] = 100;
                }
            }
        }

        msg_Mapout.header.stamp = msg_Scan.header.stamp;
        pub_Map.publish(msg_Mapout);
        cout<<"pub a cost map"<<endl;
        msg_Mapout = msg_Mapin;
    }
}



void Map_Process::InitialMap()
{
    nav_msgs::GetMap getmap;
    nav_msgs::OccupancyGrid newmap;

    if(sub_globalmap_srv.call(getmap))
    {
        /** TO DO : execute feed back action*/
        newmap = getmap.response.map;
        cout<<"global map is initialed!!!"<<endl;
        b_IsMapInitial = true;

    }
    else
    {
        ROS_ERROR("Failed to call service turtlebot_feedback");
        return;
    }

    msg_Mapin = newmap;

    map_FrameId = msg_Mapin.header.frame_id;

    f_Mapresolution = msg_Mapin.info.resolution;
    n_Mapheight = int(msg_Mapin.info.height);
    n_Mapwidth = int(msg_Mapin.info.width);
    f_Mapheight = n_Mapheight*f_Mapresolution;
    f_Mapwidth = n_Mapwidth*f_Mapresolution;
    f_MapOrigx = msg_Mapin.info.origin.position.x;
    f_MapOrigy = msg_Mapin.info.origin.position.y;


    n_diasize = int(f_robotradius/f_Mapresolution);

//    cout<<"dia size is "<<n_diasize<<endl;

    int nIndex;
    int nIndexX;
    int nIndexY;
    for(int i=0;i<newmap.data.size();i++)
    {
        if(newmap.data[i] < n_obs_threshold)
            continue;

        nIndex = i;
        nIndexX = i%int(n_Mapwidth);
        nIndexY = i/int(n_Mapwidth);

        int diasize = int(f_robotradius/f_Mapresolution);
        int start_indexX = nIndexX - n_diasize ;
        int end_indexX = nIndexX + n_diasize;

        int start_indexY = nIndexY - n_diasize;
        int end_indexY = nIndexY + n_diasize;

        if(start_indexX < 0) start_indexX = 0;
        if(start_indexY < 0) start_indexY = 0;
        if(end_indexX > n_Mapwidth) end_indexX = n_Mapwidth;
        if(end_indexY > n_Mapheight) end_indexY = n_Mapheight;

        int Index;
        int IndexX;
        int IndexY;

        for(IndexX=start_indexX; IndexX<end_indexX;IndexX++)
        {
            for(IndexY=start_indexY; IndexY<end_indexY;IndexY++)
            {
                Index = IndexY*n_Mapwidth + IndexX;
                msg_Mapin.data[Index] = 100;
            }
        }

    }

    msg_Mapout = msg_Mapin;
}

void Map_Process::scanCallback(sensor_msgs::LaserScan newscan)
{
    if(b_IsScanInitial == false)
    {       
        InitialScan(newscan);
        b_IsScanInitial = true;
        cout<<"laser scan is initialed!!!"<<endl;
    }
    msg_Scan = newscan;



    tf::StampedTransform orig_pose;

    try{
      listener_.lookupTransform("/map", scan_FrameId,
                               ros::Time(0), orig_pose);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    msg_scanpose.pose.position.x = orig_pose.getOrigin().getX();
    msg_scanpose.pose.position.y = orig_pose.getOrigin().getY();
    msg_scanpose.pose.position.z = orig_pose.getOrigin().getZ();


    msg_scanpose.pose.orientation.x = orig_pose.getRotation().getX();
    msg_scanpose.pose.orientation.y = orig_pose.getRotation().getY();
    msg_scanpose.pose.orientation.z = orig_pose.getRotation().getZ();
    msg_scanpose.pose.orientation.w = orig_pose.getRotation().getW();

}


void Map_Process::mapCallback(nav_msgs::OccupancyGrid newmap)
{
    if(b_IsMapInitial == false)
    {       
        b_IsMapInitial = true;
        cout<<"map is initialed!!!"<<endl;
    }
    msg_Mapin = newmap;
    map_FrameId = msg_Mapin.header.frame_id;

    f_Mapresolution = msg_Mapin.info.resolution;
    n_Mapheight = int(msg_Mapin.info.height);
    n_Mapwidth = int(msg_Mapin.info.width);
    f_Mapheight = n_Mapheight*f_Mapresolution;
    f_Mapwidth = n_Mapwidth*f_Mapresolution;
    f_MapOrigx = msg_Mapin.info.origin.position.x;
    f_MapOrigy = msg_Mapin.info.origin.position.y;


    n_diasize = int(f_robotradius/f_Mapresolution);

//    cout<<"dia size is "<<n_diasize<<endl;

    int nIndex;
    int nIndexX;
    int nIndexY;
    for(int i=0;i<newmap.data.size();i++)
    {
        if(newmap.data[i] < n_obs_threshold)
            continue;

        nIndex = i;
        nIndexX = i%int(n_Mapwidth);
        nIndexY = i/int(n_Mapwidth);

        int diasize = int(f_robotradius/f_Mapresolution);
        int start_indexX = nIndexX - n_diasize ;
        int end_indexX = nIndexX + n_diasize;

        int start_indexY = nIndexY - n_diasize;
        int end_indexY = nIndexY + n_diasize;

        if(start_indexX < 0) start_indexX = 0;
        if(start_indexY < 0) start_indexY = 0;
        if(end_indexX > n_Mapwidth) end_indexX = n_Mapwidth;
        if(end_indexY > n_Mapheight) end_indexY = n_Mapheight;

        int Index;
        int IndexX;
        int IndexY;

        for(IndexX=start_indexX; IndexX<end_indexX;IndexX++)
        {
            for(IndexY=start_indexY; IndexY<end_indexY;IndexY++)
            {
                Index = IndexY*n_Mapwidth + IndexX;
                msg_Mapin.data[Index] = 100;
            }
        }

    }

    msg_Mapout = msg_Mapin;
}

geometry_msgs::Point Map_Process::fromlasertomap(geometry_msgs::Point laser_point)
{
    geometry_msgs::Point map_point;
    geometry_msgs::Quaternion q = msg_scanpose.pose.orientation;

    geometry_msgs::Quaternion nq;
    nq.x = q.x/(sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w));
    nq.y = q.y/(sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w));
    nq.z = q.z/(sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w));
    nq.w = q.w/(sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w));




    float alpha = tf::getYaw(nq);
    float dx = msg_scanpose.pose.position.x;
    float dy = msg_scanpose.pose.position.y;

    map_point.x = laser_point.x * cos(alpha) - laser_point.y * sin(alpha) + dx;
    map_point.y = laser_point.x * sin(alpha) + laser_point.y * cos(alpha) + dy;

    return map_point;

}

void Map_Process::InitialScan(sensor_msgs::LaserScan newscan)
{
    scan_FrameId = newscan.header.frame_id;//newscan.header.frame_id;
    f_anglemin = newscan.angle_min;
    f_anglemax = newscan.angle_max;
    f_angle_increment = newscan.angle_increment;
    f_rangemin = newscan.range_min;
    f_rangemax = newscan.range_max;
    int i = 0;
    while(true)
    {
        float theta = f_anglemin + f_angle_increment*i;
        if(theta > f_anglemax)
            break;

        v_thetas.push_back(theta);
        i++;
    }
}


bool Map_Process::IsInMap(float fX, float fY)
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

bool Map_Process::LocatePointInMap(float fX, float fY, int *nIndexX, int *nIndexY)
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

bool Map_Process::IsInArray(int nIndexX, int nIndexY)
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


bool Map_Process::LocateIndexInArray(int nIndexX, int nIndexY, int *nIndex)
{
    if(!IsInArray(nIndexX, nIndexY))
    {
        return false;
    }

    *nIndex = nIndexY*n_Mapwidth+nIndexX;
    return true;
}


bool Map_Process::IsObstacle(int nArrayIndex)
{
    if(msg_Mapin.data[nArrayIndex] >= n_obs_threshold)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Map_Process::IsPointStateFree(float fX, float fY)
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

}
