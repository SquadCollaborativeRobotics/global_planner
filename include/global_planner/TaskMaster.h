/**
 *  TaskMaster.h
 *
 *  This class keeps track of all waypoints, goals, and dump tasks
 *      It handles sending, receiving, and updating the tasks based on the messages
 *      exchanged
 *
 *  (c) 2014
 */

#pragma once

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>

#include "Conversion.h"
#include "TaskResult.h"

#include "RobotStatusWrapper.h"
#include "GoalWrapper.h"
#include "WaypointWrapper.h"
#include "DumpWrapper.h"

#include <global_planner/GoalSeen.h>
#include <global_planner/GoalFinished.h>
#include <global_planner/WaypointFinished.h>
#include <global_planner/DumpFinished.h>

#include <global_planner/WaypointSrv.h>
#include <global_planner/DumpSrv.h>
#include <global_planner/GoalSrv.h>
#include <global_planner/GoalSrv.h>

#include <std_msgs/String.h>


class TaskMaster
{
public:

    TaskMaster();
    ~TaskMaster(){};

    // Pass in the NodeHandle, as well as the list of robots on the network (for setting up callbacks/publishers)
    bool Init(ros::NodeHandle* nh, std::map<int, Robot_Ptr > robots, std::string waypoint_filename);

    // Add goal to the goal list
    bool AddGoal(Goal_Ptr goal);
    // Add goal to the waypoint list
    bool AddWaypoint(Waypoint_Ptr waypoint);
    // Add goal to the dump list
    bool AddDump(Dump_Ptr dump);

    std::vector<Goal_Ptr> GetAvailableGoals();
    std::vector<Waypoint_Ptr> GetAvailableWaypoints();
    std::vector<Dump_Ptr> GetAvailableDumps();

    bool isFinished();

    //Clear all lists of goals, waypoints, and dumps
    bool Clear();

    // Send messages over ROS
    bool SendWaypoint(int wpID);
    bool SendGoal(int goalID);
    bool SendDump(int dumpID);

    std::map<int, Goal_Ptr > GetGoals();
    std::map<int, Waypoint_Ptr > GetWaypoints();
    std::map<int, Dump_Ptr > GetDumps();

    // *******************************
    // ROS Callbacks
    // *******************************
    bool cb_goalFinished(global_planner::GoalFinished::Request  &req,
                         global_planner::GoalFinished::Response &res);
    bool cb_waypointFinished(global_planner::WaypointFinished::Request  &req,
                             global_planner::WaypointFinished::Response &res);
    bool cb_dumpFinished(global_planner::DumpFinished::Request  &req,
                         global_planner::DumpFinished::Response &res);
    void cb_goalSeen(const global_planner::GoalSeen::ConstPtr& msg);

    void LoadWaypoints(std::string filename);
    void UpdateRobotMap(std::map< int, Robot_Ptr > new_robots);

private:
    // Initialize lists, setup callbacks, regiser services
    bool SetupTopics();
    bool RegisterServices();

    // ros NodeHandle used for publishing and tf stuff
    ros::NodeHandle* m_nh;

    /**
     * Lists of goals, waypoints, & dumps
     */
    // List of goals that need to be accomplished
    std::map<int, Goal_Ptr > m_goalMap;
    // List of waypoints that need to be accomplished
    std::map<int, Waypoint_Ptr > m_waypointMap;
    // List of dumps that need to be accomplished
    std::map<int, Dump_Ptr > m_dumpMap;


    //For now, we'll just listen to one topic for goals, waypoints, and dump results,
    //  since everything is being published with an ID anyways
    ros::Subscriber m_goalSeenSub;
    ros::Subscriber m_dumpNeededSub;


    /**
     * List of services that will communicate with the robots
     */
    // ServiceClients for the waypoints, goals, and dumps
    std::map<int, ros::ServiceClient > m_waypointClients;
    std::map<int, ros::ServiceClient > m_goalClients;
    std::map<int, ros::ServiceClient > m_dumpClients;
    std::map<int, ros::ServiceServer > m_wpFinishedService;
    std::map<int, ros::ServiceServer > m_dumpFinishedService;
    std::map<int, ros::ServiceServer > m_goalFinishedService;

    // List of robots in the system
    std::map< int, Robot_Ptr > m_robots;


    void SendSound(std::string filename);
    void SendText(std::string text);
    // Sound message publisher
    ros::Publisher m_soundPub;
    // Sound message publisher
    ros::Publisher m_textPub;
};
