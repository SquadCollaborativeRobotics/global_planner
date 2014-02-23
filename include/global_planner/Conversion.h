#pragma once

#include <ros/ros.h>

// #include "TaskMaster.h"
#include "TaskResult.h"


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
            case TaskResult::MOVE_BASE_FAILURE:
            return "Move base failed to reach goal";
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
};
