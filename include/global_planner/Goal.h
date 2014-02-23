#pragma once

#include <ros/ros.h>
#include <string>

#include <geometry_msgs/Pose.h>
#include <global_planner/GoalMsg.h>

class Goal
{

public:
    Goal(){};
    Goal(global_planner::GoalMsg& msg){ m_msg = msg; };
    ~Goal(){};

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

    void SetData(global_planner::GoalMsg& data){ m_msg = data; };
    global_planner::GoalMsg GetMessage(){ return m_msg; };
private:
    global_planner::GoalMsg m_msg;
};

typedef boost::shared_ptr<Goal> Goal_Ptr;
