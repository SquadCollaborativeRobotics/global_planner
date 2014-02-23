#pragma once

#include <ros/ros.h>
#include <string>

#include <geometry_msgs/Pose.h>

#include <global_planner/DumpMsg.h>

class DumpWrapper
{

public:
    DumpWrapper(){};
    DumpWrapper(global_planner::DumpMsg& msg){ m_msg = msg; };
    ~DumpWrapper(){};

    void SetID(int id){ m_msg.id = id; };
    void SetTime(ros::Time time){ m_msg.time = time; };
    void SetPose1(geometry_msgs::Pose pose1){ m_msg.pose1 = pose1; };
    void SetPose2(geometry_msgs::Pose pose2){ m_msg.pose2 = pose2; };
    void SetRobot1(int robotID1){ m_msg.robotID1 = robotID1; };
    void SetRobot2(int robotID2){ m_msg.robotID2 = robotID2; };

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
    geometry_msgs::Pose GetPose1() {return m_msg.pose1; };
    geometry_msgs::Pose GetPose2() {return m_msg.pose2; };
    int GetRobot1(){ return m_msg.robotID1; };
    int GetRobot2(){ return m_msg.robotID2; };

    void SetData(global_planner::DumpMsg& data){ m_msg = data; };
    global_planner::DumpMsg GetMessage(){ return m_msg; };
private:
    global_planner::DumpMsg m_msg;
};

typedef boost::shared_ptr<DumpWrapper> Dump_Ptr;
