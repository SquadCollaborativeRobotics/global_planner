#pragma once

#include <ros/ros.h>
#include "TaskMaster.h"
#include "Conversions.h"
#include "global_planner/RobotStatus.h"

class GlobalPlanner
{
public:
    GlobalPlanner();
    ~GlobalPlanner();

    // Get information from a robot through the robots "get info" service
    //GetRobotInfo(int id);

    // Call setup functions
    bool Init(ros::NodeHandle& nh);

    // Executive function
    void Execute();

    // System finished
    void Finished();

    std::vector< boost::shared_ptr<Goal> > GetGoalList();

private:
    // setup callbacks, regiser services, load waypoints...
    bool SetupCallbacks();
    bool RegisterServices();
    bool LoadWaypoints(std::string filename);

    // get robot information
    void cb_robotStatus(const global_planner::RobotStatus::ConstPtr& msg);

    // Send request to all listening robots that
    // they should send out their information to be added to the list of bots
    int FindRobots();

    // Pointer to a registered ros NodeHandle
    ros::NodeHandle *m_nh;

    // List of robots during this run of the program
    std::map<int, Robot_Ptr > m_robots;

    // Task Master
    TaskMaster m_tm;
};
