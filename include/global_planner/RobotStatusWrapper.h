/**
 *  RobotStatusWrapper.h
 *
 *  RobotStatusWrapper is a wrapper class for the RobotStatus message, providing useful functions
 *      for getting and setting data
 *
 *  (c) 2014
 */

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <global_planner/RobotStatus.h>
#include <global_planner/RobotState.h>

#include <string>

class RobotStatusWrapper
{

public:
    RobotStatusWrapper(){};
    ~RobotStatusWrapper(){};

    /************************************************************************
     ** BEGIN DATA section
     */

    void SetID(int id){ m_status.id = id; };
    void SetName(std::string name){ m_status.name = name; };
    void SetPose(geometry_msgs::Pose pose){ m_status.pose = pose; };
    void SetTwist(geometry_msgs::Twist twist){ m_status.twist = twist; };
    void SetStorageCapacity(int cap){ m_status.storage_capacity = cap; };
    void SetStorageUsed(int amountUsed){ m_status.storage_used = amountUsed; };

    void IncrementStorageUsed(int amountToIncrement = 1)
    {
        if (GetStorageAvailable() > 0)
        {
            m_status.storage_used = m_status.storage_used + 1;
        }
        else
        {
            ROS_ERROR_STREAM("Cannot add any more stuff to this robot - storage available = " << GetStorageAvailable());
        }
    };

    void SetType(RobotState::Type type){ m_status.type = type; };
    void SetTaskID(int taskID){ m_status.taskID = taskID; };

    std::string ToString()
    {
        std::stringstream ss;
        geometry_msgs::Pose p = GetPose();
        geometry_msgs::Twist t = GetTwist();
        ss<<"robot id = "<<GetID()<<" State: "<<RobotState::ToString(GetState())
        <<" | amount filled: "<<GetStorageUsed()<<" | capacity: "<<GetStorageCapacity()
        <<" current task = "<<GetTaskID()
        <<" -- position: "<< p.position.x << ' ' << p.position.y << ' ' << p.orientation.z << ' ' << p.orientation.w
        <<" velocity: "<< t.linear.x << ' ' << t.linear.y << ' ' << t.angular.z;
        return ss.str();
    }


    /******************
    ** state stuff...
    *...
    *...
    *******************/
    RobotState::State GetState() { return RobotState::IntToRobotState(m_status.state); };
    void SetState(RobotState::State state) { m_status.state = RobotState::RobotStateToInt(state); };

    //Getters

    int GetID() { return m_status.id; };
    std::string GetName() { return m_status.name; };
    geometry_msgs::Pose GetPose() {return m_status.pose; };
    geometry_msgs::Twist GetTwist() { return m_status.twist; };
    int GetStorageCapacity(){ return m_status.storage_capacity; };
    int GetStorageUsed(){ return m_status.storage_used; };
    int GetStorageAvailable(){ return m_status.storage_capacity - m_status.storage_used; };
    RobotState::Type GetType(){ return static_cast<RobotState::Type>(m_status.type); };
    int GetTaskID(){ return m_status.taskID; };

    void SetData(global_planner::RobotStatus data){ m_status = data; };

    global_planner::RobotStatus GetMessage(){ return m_status; };

    /**********************************************************************
    **
    **END DATA SECTION
    **
    */

private:
    global_planner::RobotStatus m_status;
};

typedef boost::shared_ptr<RobotStatusWrapper> Robot_Ptr;
