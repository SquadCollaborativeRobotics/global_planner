#pragma once

#include <ros/ros.h>

// #include "TaskMaster.h"
#include "TaskResult.h"
#include "move_base_msgs/MoveBaseGoal.h"


class Conversion
{
public:
    Conversion();
    ~Conversion();

    static const std::string TaskResultToString(TaskResult::Status status)
    {
        switch(status)
        {
            case TaskResult::SUCCESS:
            return "Success";
            case TaskResult::FAILURE:
            return "General Failure";
            case TaskResult::INPROGRESS:
            return "Task still in progress";
            case TaskResult::FORCE_STOP:
            return "Task was force stopped by global planner";
            case TaskResult::NAVSTACK_FAILURE:
            return "Nav Stack failed to reach goal";
            default:
            return "Unknown result code";
        }
    }

    //Callback from robot's task converted to a TaskManager "status" enum
    static const TaskResult::Status IntToTaskResult(int num)
    {
        return static_cast<TaskResult::Status>(num);
    }

    // Conver from a TaskResult "status" enum to an integer
    static const int TaskResultToInt(TaskResult::Status s)
    {
        return static_cast<int>(s);
    }

    static const geometry_msgs::Pose SetPose(double x, double y, double rz, double rw)
    {
        geometry_msgs::Pose p;
        p.position.x = x;
        p.position.y = y;
        p.orientation.z = rz;
        p.orientation.w = rw;
        return p;
    }

    static const std::string PoseToString(geometry_msgs::Pose pose)
    {
        std::stringstream ss;
        ss << "x = "<<pose.position.x << ", y = "<<pose.position.y<<", rz = "<<pose.orientation.z<<", rw = "<<pose.orientation.w;
        return ss.str();
    }

    static const move_base_msgs::MoveBaseGoal PoseToMoveBaseGoal(const geometry_msgs::Pose& pose)
    {
        //TODO: this
    }
};
