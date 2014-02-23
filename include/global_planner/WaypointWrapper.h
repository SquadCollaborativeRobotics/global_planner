#pragma once

#include <ros/ros.h>
#include <string>

#include <geometry_msgs/Pose.h>

#include <global_planner/WaypointMsg.h>

#include "Conversion.h"


class WaypointWrapper
{

public:
    WaypointWrapper(){};
    ~WaypointWrapper(){};

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
    bool GetCompleted(){ return m_msg.status == TaskResult::SUCCESS; };
    bool GetAvailable(){ return m_msg.status == TaskResult::AVAILABLE; };
    bool GetInProgress(){ return m_msg.status == TaskResult::INPROGRESS; };
    bool GetFailed(){ return m_msg.status == TaskResult::FAILURE; };

    void SetStatus(TaskResult::Status s){ m_msg.status = Conversion::TaskResultToInt(s); };

    void SetData(global_planner::WaypointMsg& data){ m_msg = data; };
    global_planner::WaypointMsg GetMessage(){ return m_msg; };
private:
    global_planner::WaypointMsg m_msg;
};

typedef boost::shared_ptr<WaypointWrapper> Waypoint_Ptr;
