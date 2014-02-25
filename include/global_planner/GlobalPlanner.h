/**
 *  GlobalPlanner.h
 *
 *  This class wraps up all global planning functionality
 *      It's main functionality is maintaining the list of robots, along with their
 *      status.  The class contains a TaskMaster member that keeps track of all
 *      tasks being created/updated
 *
 *  The job of Global Planner is to select which tasks to give to which robots
 *
 *  (c) 2014
 */

#pragma once

#include <ros/ros.h>
#include "TaskMaster.h"
#include "Conversion.h"
#include <std_msgs/Empty.h>
#include <global_planner/RobotStatusWrapper.h>
#include <global_planner/RobotState.h>
#include <global_planner/SoundMsg.h>

class GlobalPlanner
{
public:
    GlobalPlanner();
    ~GlobalPlanner();

    // Call setup functions
    bool Init(ros::NodeHandle* nh);

    void Display();

    // Executive function
    void Execute();

    // System finished
    void Finished();

    std::map<int, Goal_Ptr > GetGoals() { return m_tm.GetGoals(); };
    std::map<int, Waypoint_Ptr > GetWaypoints() { return m_tm.GetWaypoints(); };
    std::map<int, Dump_Ptr > GetDumps() { return m_tm.GetDumps(); };

    std::vector<Robot_Ptr> GetAvailableRobots();

    int GetBestBinBot(int idOfRobotThatNeedsIt);
    int GetBestCollectorbot(int goalID);
    int GetBestSearchBot(int wpID);


    void SendSound(std::string filename, int num_times);
    void SendSound(std::string filename);

private:
    // setup callbacks, regiser services, load waypoints...
    bool SetupCallbacks();
    bool RegisterServices();

    // get robot information
    void cb_robotStatus(const global_planner::RobotStatus::ConstPtr& msg);

    // Send request to all listening robots that
    // they should send out their information to be added to the list of bots
    int FindRobots();

    // Pointer to a registered ros NodeHandle
    ros::NodeHandle *m_nh;

    // List of robots during this run of the program
    std::map<int, Robot_Ptr > m_robots;

    //Subscriber to robot status callbacks
    ros::Subscriber m_robotSub;

    // E-Stop Publisher
    ros::Publisher m_eStopPub;

    // E-Stop Publisher
    ros::Publisher m_soundPub;

    // Task Master
    TaskMaster m_tm;

    ros::Time m_lastDisplay;
};
