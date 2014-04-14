/**
 *  WaypointWrapper.h
 *
 *  WaypointWrapper is a wrapper class for the WaypointMsg, providing useful functions
 *      for getting and setting data
 *
 *  (c) 2014
 */

#pragma once

#include <ros/ros.h>
#include <string>

#include <geometry_msgs/Pose.h>

#include <global_planner/WaypointMsg.h>

#include "Conversion.h"


class WaypointWrapper
{

public:
    WaypointWrapper()
    {
        Init(-1,-1, TaskResult::UNINITIALIZED, 0,0,0,0);
    };
    WaypointWrapper(int id, double x, double y, double z, double w)
    {
        Init(id, -1, TaskResult::AVAILABLE, x,y,z,w);
    };

    ~WaypointWrapper(){};

    void Init(int id, int robotID, TaskResult::Status status, double x, double y, double rz, double rw)
    {
        SetID(id);
        SetTime(ros::Time::now());
        geometry_msgs::Pose p = Conversion::SetPose(x,y,rz,rw);
        SetPose(p);
        SetRobot(robotID);
        SetStatus(status);
    }

    void SetID(int id){ m_msg.id = id; };
    void SetTime(ros::Time time){ m_msg.time = time; };
    void SetPose(geometry_msgs::Pose pose){ m_msg.pose = pose; };
    void SetRobot(int robotID){ m_msg.robotID = robotID; };
    void SetStatus(TaskResult::Status s){ m_msg.status = TaskResult::ToInt(s); };

    std::string GetStatusMessage()
    {
        int status = GetStatus();
        return (GetCompleted() ? "DONE       " :
                                 (GetInProgress() ? "IN PROGRESS" :
                                                    (GetAvailable() ? "AVAILABLE  " : "FAILED     ")));
    }

    std::string ToString()
    {
        std::stringstream ss;
        geometry_msgs::Pose p = GetPose();
        ss << "Waypoint(" << GetID() << ") "
            << GetStatusMessage()
            <<" [x:"<< p.position.x << ", y:" << p.position.y << ", w:" << p.orientation.w << ']';
        // << " | updated at time: "<<GetTime()<<" -- "<< p.position.x << ' ' << p.position.y << ' ' << p.orientation.z << ' ' << p.orientation.w;
        return ss.str();
    }

    std::string ToShortString()
    {
        std::stringstream ss;
        int status = GetStatus();
        ss << "WP(" << GetID() << ") "
            << GetStatusMessage();
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
    geometry_msgs::Pose GetPose() {return m_msg.pose; };
    int GetRobot(){ return m_msg.robotID; };
    bool GetCompleted(){ return m_msg.status == TaskResult::SUCCESS; };
    bool GetAvailable(){ return m_msg.status == TaskResult::AVAILABLE; };
    bool GetInProgress(){ return m_msg.status == TaskResult::INPROGRESS; };
    bool GetFailed(){ return m_msg.status == TaskResult::FAILURE; };
    int GetStatus(){ return m_msg.status; };

    void SetData(global_planner::WaypointMsg& data){ m_msg = data; };
    global_planner::WaypointMsg GetMessage(){ return m_msg; };
private:
    global_planner::WaypointMsg m_msg;
};

typedef boost::shared_ptr<WaypointWrapper> Waypoint_Ptr;
