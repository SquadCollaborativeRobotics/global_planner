#pragma once

#include <ros/ros.h>
#include <string>

#include <geometry_msgs/Pose.h>
#include <global_planner/GoalMsg.h>

#include "Conversion.h"

class GoalWrapper
{
public:
    GoalWrapper()
    {
    };
    ~GoalWrapper(){};

    void SetID(int id){ m_msg.id = id; };
    void SetTime(ros::Time time){ m_msg.time = time; };
    void SetPose(geometry_msgs::Pose pose){ m_msg.pose = pose; };
    void SetRobot(int robotID){ m_msg.robotID = robotID; };
    void SetStatus(TaskResult::Status s){ m_msg.status = Conversion::TaskResultToInt(s); };


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

    //Getter

    int GetID() { return m_msg.id; };
    ros::Time GetTime(){ return m_msg.time; };
    geometry_msgs::Pose GetPose() {return m_msg.pose; };
    int GetRobot(){ return m_msg.robotID; };
    bool GetCompleted(){ return m_msg.status == TaskResult::SUCCESS; };
    bool GetAvailable(){ return m_msg.status == TaskResult::AVAILABLE; };
    bool GetInProgress(){ return m_msg.status == TaskResult::INPROGRESS; };
    bool GetFailed(){ return m_msg.status == TaskResult::FAILURE; };
    bool GetStatus(){ return m_msg.status; };

    global_planner::GoalMsg GetMessage(){ return m_msg; };

    static int num_goals_generated;
private:
    global_planner::GoalMsg m_msg;
};

typedef boost::shared_ptr<GoalWrapper> Goal_Ptr;
