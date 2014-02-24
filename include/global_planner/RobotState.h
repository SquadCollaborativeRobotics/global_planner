#pragma once

class RobotState
{
public:
    enum State
    {
        WAITING = 0,
        NAVIGATING = 1,
        DUMPING = 2,
        COLLECTING = 3,
    };

    static const int RobotStateToInt(RobotState::State s)
    {
        return static_cast<int>(s);
    }

    static const RobotState::State IntToRobotState(int s)
    {
        return static_cast<RobotState::State>(s);
    }
};
