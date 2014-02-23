#pragma once

#include <ros/ros.h>

#include "global_planner/Goal.h"
#include "global_planner/Waypoint.h"
#include "global_planner/Dump.h"
#include "global_planner/GoalMsg.h"
#include "global_planner/WaypointMsg.h"
#include "global_planner/DumpMsg.h"

#include "global_planner/TaskMaster.h"

class Conversion
{
public:
    Conversion();
    ~Conversion();

    static const std::string ReturnStatusToString(TaskResultStatus status)
    {
        switch(status)
        {
            case SUCCESS:
            return "Success";
            case FAILURE:
            return "General Failure";
            case INPROGRESS:
            return "Task still in progress";
            case FORCE_STOP:
            return "Task was force stopped by global planner";
            case MOVE_BASE_FAILURE:
            return "Move base failed to reach goal";
            default:
            return "Unknown result code";
        }
    }

    //Callback from robot's task converted to a TaskManager "status" enum
    static const TaskResultStatus ReturnIntToStatus(int num)
    {
        return static_cast<TaskResultStatus>(num);
    }
};
