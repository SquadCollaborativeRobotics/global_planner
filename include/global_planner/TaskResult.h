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
        UNINITIALIZED = 99,
    };
};
