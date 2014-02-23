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
        MOVE_BASE_FAILURE = 5,
        UNINITIALIZED = 99,
    };
};