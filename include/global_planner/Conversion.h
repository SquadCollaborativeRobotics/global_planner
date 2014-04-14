/**
 *  Conversion.h
 *
 *  This helper class contains useful functions for converting between
 *      different data types
 *
 *  (c) 2014
 */

#pragma once

#include <ros/ros.h>

// #include "TaskMaster.h"
#include "TaskResult.h"
#include <move_base_msgs/MoveBaseGoal.h>


class Conversion
{
public:
    Conversion();
    ~Conversion();

    static const geometry_msgs::Pose SetPose(double x, double y, double rz, double rw)
    {
        geometry_msgs::Pose p;
        p.position.x = x;
        p.position.y = y;
        p.orientation.z = rz;
        p.orientation.w = rw;
        return p;
    }

    static const std::string PoseToString(geometry_msgs::Pose pose)
    {
        std::stringstream ss;
        ss << "x = "<<pose.position.x << ", y = "<<pose.position.y<<", rz = "<<pose.orientation.z<<", rw = "<<pose.orientation.w;
        return ss.str();
    }

    static const move_base_msgs::MoveBaseGoal PoseToMoveBaseGoal(const geometry_msgs::Pose& pose)
    {
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = pose.position.x;
        goal.target_pose.pose.position.y = pose.position.y;
        goal.target_pose.pose.orientation.z = pose.orientation.z;
        goal.target_pose.pose.orientation.w = pose.orientation.w;

        return goal;
    }

    static const std::string RobotIDToServiceName(int id)
    {
        std::stringstream ss;
        ss << "/robot_status/"<<id;
        return ss.str();
    }
    static const std::string RobotIDToWaypointTopic(int id)
    {
        std::stringstream ss;
        ss << "/waypoints/"<<id;
        return ss.str();
    }
    static const std::string RobotIDToGoalTopic(int id)
    {
        std::stringstream ss;
        ss << "/goals/"<<id;
        return ss.str();
    }
    static const std::string RobotIDToDumpTopic(int id)
    {
        std::stringstream ss;
        ss << "/dumps/"<<id;
        return ss.str();
    }
};
