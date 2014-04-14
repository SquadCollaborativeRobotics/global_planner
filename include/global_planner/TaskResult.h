/**
 *  TaskResult.h
 *
 *  This class is an enumeration that is used to simplify how we represent the
 *      return status from a task, using names instead of integers because
 *      this is easier for both readability and portability
 *
 *  (c) 2014
 */

#pragma once

class TaskResult
{
public:
    enum Status
    {
        SUCCESS = 0,
        AVAILABLE = 1,
        INPROGRESS = 2,
        FAILURE = 3,
        FORCE_STOP = 4,
        NAVSTACK_FAILURE = 5,
        COMM_FAILURE = 10,
        UNINITIALIZED = 99,
    };

    static const std::string ToString(TaskResult::Status s)
    {
        switch (s)
        {
            case SUCCESS:
                return "SUCCESS";
            case AVAILABLE:
                return "AVAILABLE";
            case INPROGRESS:
                return "INPROGRESS";
            case FAILURE:
                return "FAILURE";
            case FORCE_STOP:
                return "FORCE_STOP";
            case NAVSTACK_FAILURE:
                return "NAVSTACK_FAILURE";
            case COMM_FAILURE:
                return "COMM_FAILURE";
            case UNINITIALIZED:
                return "UNINITIALIZED";
            default:
                return"ERROR: UNKNOWN RESULT";
        }
    }

    //Callback from robot's task converted to a TaskManager "status" enum
    static const TaskResult::Status ToEnum(int num)
    {
        return static_cast<TaskResult::Status>(num);
    }

    // Conver from a TaskResult "status" enum to an integer
    static const int ToInt(TaskResult::Status s)
    {
        return static_cast<int>(s);
    }
};
