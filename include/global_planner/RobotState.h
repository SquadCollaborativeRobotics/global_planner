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
        NONE = 0,
        WAITING = 1,
        WAITING_TAG_SPOTTED = 2,
        WAITING_TAG_FINISHED = 3,
        // WAITING_FINISHED = 9,
        NAVIGATING = 10,
        NAVIGATING_TAG_SPOTTED = 12,
        NAVIGATING_TAG_FINISHED = 13,
        // NAVIGATING_FINISHED = 19,
        DUMPING = 20,
        DUMPING_FINISHED = 29,
        // COLLECTING_FINISHED = 39,
        UNINITIALIZED = 99,
        ESTOP = 100
    };

    static const int ToInt(RobotState::State s)
    {
        return static_cast<int>(s);
    }

    static const RobotState::State ToRobotState(int s)
    {
        return static_cast<RobotState::State>(s);
    }

    static const std::string ToString(RobotState::State s)
    {
        switch(s)
        {
            case RobotState::NONE:
                return "NONE";
            case RobotState::WAITING:
                return "WAITING";
            case RobotState::WAITING_TAG_SPOTTED:
                return "WAITING_TAG_SPOTTED";
            case RobotState::WAITING_TAG_FINISHED:
                return "WAITING_TAG_FINISHED";
            case RobotState::NAVIGATING:
                return "NAVIGATING";
            case RobotState::NAVIGATING_TAG_SPOTTED:
                return "NAVIGATING_TAG_SPOTTED";
            case RobotState::NAVIGATING_TAG_FINISHED:
                return "NAVIGATING_TAG_FINISHED";
            case RobotState::DUMPING:
                return "DUMPING";
            case RobotState::DUMPING_FINISHED:
                return "DUMPING_FINISHED";
            case RobotState::ESTOP:
                return "ESTOP";
            case RobotState::UNINITIALIZED:
                return "UNINITIALIZED";
            default:
                return "UNKNOWN robot state (or not implemented in ToString func";
        }
    }
};
