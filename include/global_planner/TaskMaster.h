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
#include "WaypointWrapper.h"
#include "DumpWrapper.h"

#include <global_planner/GoalSeen.h>
#include <global_planner/WaypointFinished.h>
#include <global_planner/DumpFinished.h>

#include <global_planner/WaypointSrv.h>
#include <global_planner/DumpSrv.h>

#include <std_msgs/String.h>

#define WAYPOINT_START_ID 1000 // Start id for waypoints read from file

class TaskMaster
{
public:

    TaskMaster();
    ~TaskMaster(){};

    // Pass in the NodeHandle, as well as the list of robots on the network (for setting up callbacks/publishers)
    bool Init(ros::NodeHandle* nh, std::string waypoint_filename);

    // Add waypoint to the waypoint list
    bool AddWaypoint(Waypoint_Ptr waypoint);
    // Add dump to the dump list
    bool AddDump(Dump_Ptr dump);

    std::vector<Waypoint_Ptr> GetAvailableWaypoints();
    std::vector<Waypoint_Ptr> GetAvailableGoals();
    std::vector<Dump_Ptr> GetAvailableDumps();

    bool IsWaypoint(int taskID);
    bool IsGoal(int taskID);
    bool IsAvailable(int taskID);
    bool IsInProgress(int taskID);

    bool isFinished();

    //Clear all lists of goals, waypoints, and dumps
    bool Clear();

    bool ResetFailedWaypoints();

    // Send messages over ROS
    bool SendWaypoint(int wpID);
    int SendDump(int dumpID);

    std::map<int, Waypoint_Ptr > GetWaypoints();
    std::map<int, Waypoint_Ptr > GetGoals();
    std::map<int, Dump_Ptr > GetDumps();

    // *******************************
    // ROS Callbacks
    // *******************************
    bool cb_waypointFinished(global_planner::WaypointFinished::Request  &req,
                             global_planner::WaypointFinished::Response &res);
    bool cb_dumpFinished(global_planner::DumpFinished::Request  &req,
                         global_planner::DumpFinished::Response &res);
    void cb_goalSeen(const global_planner::GoalSeen::ConstPtr& msg);

    void LoadWaypoints(std::string filename);
    bool AdvertiseServices(int robotID);
    bool RegisterClients(int robotID);

private:
    // Initialize lists, setup callbacks, regiser services
    bool SetupTopics();

    // ros NodeHandle used for publishing and tf stuff
    ros::NodeHandle* m_nh;

    /**
     * Lists of goals, waypoints, & dumps
     */
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
    // ServiceClients for the waypoints and dumps
    std::map<int, ros::ServiceClient > m_waypointClients;
    std::map<int, ros::ServiceClient > m_dumpClients;
    std::map<int, ros::ServiceServer > m_wpFinishedService;
    std::map<int, ros::ServiceServer > m_dumpFinishedService;

    void SendSound(std::string filename);
    void SendText(std::string text);
    // Sound message publisher
    ros::Publisher m_soundPub;
    // Sound message publisher
    ros::Publisher m_textPub;

    int m_timesResettingWaypoints;
};
