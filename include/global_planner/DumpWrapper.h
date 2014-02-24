/**
 *  DumpWrapper.h
 *
 *  DumpWrapper is a wrapper class for the DumpMsg, providing useful functions
 *      for getting and setting data
 *
 *  (c) 2014
 */


#pragma once

#include <ros/ros.h>
#include <string>

#include <geometry_msgs/Pose.h>

#include <global_planner/DumpMsg.h>

#include "Conversion.h"

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
    void SetStatus(TaskResult::Status s){ m_msg.status = Conversion::TaskResultToInt(s); };

    std::string ToString()
    {
        std::stringstream ss;
        geometry_msgs::Pose p1 = GetPose1();
        geometry_msgs::Pose p2 = GetPose2();
        ss<<"Dump id = "<<GetID()<<" | status = "<<GetStatus()<<" | updated at time: "<<GetTime()<<" -- Robot1 = "
        <<GetRobot1()<<" | Robot2 = "<<GetRobot2()<<" | Pose 1: "
        << p1.position.x << ' ' << p1.position.y << ' ' << p1.orientation.z << ' ' << p1.orientation.w<<
        "Pose 2: "<< p2.position.x << ' ' << p2.position.y << ' ' << p2.orientation.z << ' ' << p2.orientation.w;
        return ss.str();
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

    bool GetCompleted(){ return m_msg.status == TaskResult::SUCCESS; };
    bool GetAvailable(){ return m_msg.status == TaskResult::AVAILABLE; };
    bool GetInProgress(){ return m_msg.status == TaskResult::INPROGRESS; };
    bool GetFailed(){ return m_msg.status == TaskResult::FAILURE; };
    int GetStatus(){ return m_msg.status; };


    void SetData(global_planner::DumpMsg& data){ m_msg = data; };
    global_planner::DumpMsg GetMessage(){ return m_msg; };
private:
    global_planner::DumpMsg m_msg;
};

typedef boost::shared_ptr<DumpWrapper> Dump_Ptr;
