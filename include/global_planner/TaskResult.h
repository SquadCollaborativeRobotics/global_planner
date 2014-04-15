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
        FORCE_STOP = 4,
        FAILURE = 3,
        NAVSTACK_FAILURE = 5,
        COMM_FAILURE = 10,
        DUMP_HALF_DONE = 50, 
        DUMP_FINISHED = 51, // AVAILABLE > INPROGRESS > DUMP_HALF_DONE > DUMP_FINISHED > SUCCESS
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
};
