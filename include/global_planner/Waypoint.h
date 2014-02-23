#pragma once

#include <ros/ros.h>
#include <string>

#include <geometry_msgs/Pose.h>

#include <global_planner/WaypointMsg.h>

class Waypoint
{

public:
    Waypoint(){};
    Waypoint(global_planner::WaypointMsg& msg){ m_msg = msg; };
    ~Waypoint(){};

    void SetID(int id){ m_msg.id = id; };
    void SetTime(ros::Time time){ m_msg.time = time; };
    void SetPose(geometry_msgs::Pose pose){ m_msg.pose = pose; };
    void SetRobot(int robotID){ m_msg.robotID = robotID; };

    void Print()
    {
        std::stringstream ss;
        ss<<GetID()<<":"<<GetTime()<<" -- "<<std::endl;
        ROS_INFO_STREAM(ss.str());
    }

    /******************
    ** state stuff...
    *...
    *...
    *******************/

    //Getters

    int GetID() { return m_msg.id; };
    ros::Time GetTime(){ return m_msg.time; };
    geometry_msgs::Pose GetPose() {return m_msg.pose; };
    int GetRobot(){ return m_msg.robotID; };

    void SetData(global_planner::WaypointMsg& data){ m_msg = data; };
    global_planner::WaypointMsg GetMessage(){ return m_msg; };
private:
    global_planner::WaypointMsg m_msg;
};

typedef boost::shared_ptr<Waypoint> Waypoint_Ptr;
