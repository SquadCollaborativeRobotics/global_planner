#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <global_planner/RobotStatus.h>

#include <string>

class Robot
{

public:
    Robot(){};
    ~Robot(){};

    void SetID(int id){ m_status.id = id; };
    void SetName(std::string name){ m_status.name = name; };
    void SetPose(geometry_msgs::Pose pose){ m_status.pose = pose; };
    void SetTwist(geometry_msgs::Twist twist){ m_status.twist = twist; };
    void SetStorageCapacity(int cap){ m_status.storage_capacity = cap; };
    void SetStorageUsed(int amountUsed){ m_status.storage_used = amountUsed; };
    void SetType(bool type){ m_status.type = type; };

    void Print()
    {
        std::stringstream ss;
        ss<<GetID()<<":"<<GetName()<<" -- "<<std::endl;
        ROS_INFO_STREAM(ss.str());
    }

    /******************
    ** state stuff...
    *...
    *...
    *******************/

    //Getters

    int GetID() { return m_status.id; };
    std::string GetName() { return m_status.name; };
    //TODO: Fix?
    int GetState() { return m_status.state; };
    geometry_msgs::Pose GetPose() {return m_status.pose; };
    geometry_msgs::Twist GetTwist() { return m_status.twist; };
    int GetStorageCapacity(){ return m_status.storage_capacity; };
    int GetStorageUsed(){ return m_status.storage_used; };
    bool GetType(){ return m_status.type; };

    void SetData(global_planner::RobotStatus& data){ m_status = data; };
    global_planner::RobotStatus GetMessage(){ return m_status; };
private:
    global_planner::RobotStatus m_status;
};

typedef boost::shared_ptr<Robot> Robot_Ptr;
