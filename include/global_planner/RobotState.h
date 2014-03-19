/**
 *  RobotState.h
 *
 *  This class is an enumeration that is used to represent different states and the type
 *      of the robot during operation
 *
 *  (c) 2014
 */

#pragma once

class RobotState
{
public:
    enum Type
    {
        ANY = 0,
        COLLECTOR_BOT = 1,
        BIN_BOT = 2
    };

    static const int RobotTypeToInt(RobotState::Type t)
    {
        return static_cast<int>(t);
    }
    
    static const RobotState::Type IntToRobotType(int t)
    {
        return static_cast<RobotState::Type>(t);
    }

    enum State
    {
        WAITING = 0,
        WAITING_FINISHED = 9,
        NAVIGATING = 10,
        NAVIGATING_FINISHED = 19,
        DUMPING = 20,
        DUMPING_FINISHED = 29,
        COLLECTING = 30,
        COLLECTING_FINISHED = 39,
    };

    static const int RobotStateToInt(RobotState::State s)
    {
        return static_cast<int>(s);
    }

    static const RobotState::State IntToRobotState(int s)
    {
        return static_cast<RobotState::State>(s);
    }

    static const std::string ToString(RobotState::State s)
    {
        switch(s)
        {
            case RobotState::WAITING:
            return "WAITING";
            case RobotState::NAVIGATING:
            return "NAVIGATING";
            case RobotState::DUMPING:
            return "DUMPING";
            case RobotState::COLLECTING:
            return "COLLECTING";
            default:
            return "UNKNOWN robot state";
        }
    }
};
