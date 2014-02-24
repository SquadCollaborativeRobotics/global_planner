#pragma once

class RobotState
{
public:
    enum State
    {
        WAITING = 0,
        NAVIGATING = 10,
        DUMPING = 20,
        COLLECTING = 30,
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
