#pragma once

#include <ros/ros.h>

#include "global_planner/Goal.h"
#include "global_planner/Waypoint.h"
#include "global_planner/Dump.h"
#include "global_planner/GoalMsg.h"
#include "global_planner/WaypointMsg.h"
#include "global_planner/DumpMsg.h"

#include "TaskMaster.h"
#include "TaskResult.h"


class Conversion
{
public:
    Conversion();
    ~Conversion();

    static const std::string ReturnStatusToString(TaskResult::Status status)
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
            case TaskResult::MOVE_BASE_FAILURE:
            return "Move base failed to reach goal";
            default:
            return "Unknown result code";
        }
    }

    //Callback from robot's task converted to a TaskManager "status" enum
    static const TaskResult::Status ReturnIntToStatus(int num)
    {
        return static_cast<TaskResult::Status>(num);
    }
};
