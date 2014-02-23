#pragma once

class TaskResult
{
public:
    enum Status
    {
        SUCCESS = 0,
        FAILURE = 1,
        INPROGRESS = 2,
        FORCE_STOP = 3,
        MOVE_BASE_FAILURE = 4,
    };
};