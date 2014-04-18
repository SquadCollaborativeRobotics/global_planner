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
#include <geometry_msgs/Pose.h>
#include <global_planner/RobotStatusWrapper.h>
#include <global_planner/RobotState.h>
#include <global_planner/RobotStatusSrv.h>
#include <global_planner/SetRobotStatusSrv.h>
#include <global_planner/SetTrashSrv.h>
#include <std_msgs/String.h>
#include <boost/thread/mutex.hpp>

#define NO_ROBOT_FOUND -1 // Robot ID -1 is no robot found
#define NO_WAYPOINT_FOUND -1 // Waypoint ID -1 is no robot found
#define NO_GOAL_FOUND -1 // Waypoint ID -1 is no robot found
#define MAX_DIST 1000000 // Hardcoded for robot search routine for now, 1,000 km is a reasonable for this demo

class GlobalPlanner
{
public:
    enum PLANNER_TYPE
    {
        PLANNER_NAIVE,
        PLANNER_CLOSEST_ROBOT,
        PLANNER_CLOSEST_WAYPOINT
    };
    GlobalPlanner();
    ~GlobalPlanner();

    // Call setup functions
    bool Init(ros::NodeHandle* nh);

    void Display();

    // Called when starting (for statistics)
    void Start();

    // Executive function
    void Execute();

    // System finished
    void Finished();

    std::map<int, Goal_Ptr > GetGoals() { return m_tm.GetGoals(); };
    std::map<int, Waypoint_Ptr > GetWaypoints() { return m_tm.GetWaypoints(); };
    std::map<int, Dump_Ptr > GetDumps() { return m_tm.GetDumps(); };

    std::vector<Robot_Ptr> GetAvailableRobots();
    std::vector<Robot_Ptr> GetAvailableRobots(int available_storage);

    int GetBestBinBot(int idOfRobotThatNeedsIt);
    int GetBestCollectorbot(int goalID);
    int GetBestSearchBot(int wpID);

    // Planners
    void PlanNNRobot();
    void PlanNNWaypoint();
    void PlanNaive();

    // Search algorithms
    int GetFirstAvailableBot();
    int GetFirstAvailableBot(RobotState::Type type);

    int GetRobotClosestToRobot(int robotID, RobotState::Type type);
    int GetRobotClosestToWaypoint(int waypointID, RobotState::Type type);
    int GetRobotClosestToGoal(int goalID, RobotState::Type type);
    int GetRobotClosestToPose(geometry_msgs::Pose pose, RobotState::Type type);
    int GetWaypointClosestToRobot(int robot_id);

    bool isFinished();
    bool AssignRobotWaypoint(int robot_id, int waypoint_id);
    bool AssignRobotsDump(int collector_robot_id, int bin_robot_id, int dump_id);
    bool AssignRobotGoal(int robot_id, int goal_id);
    double TimeSinceStart();

    void SendText(std::string text);
    void SendSound(std::string filename);

private:
    // setup callbacks, regiser services, load waypoints...
    bool SetupCallbacks();

    // get robot information
    void cb_robotStatus(const global_planner::RobotStatus::ConstPtr& msg);

    // Gets x/y 2D distance between two poses
    double Get2DPoseDistance(geometry_msgs::Pose a, geometry_msgs::Pose b);

    void QueryRobots();

    // Pointer to a registered ros NodeHandle
    ros::NodeHandle *m_nh;

    // List of robots during this run of the program
    std::map<int, Robot_Ptr > m_robots;

    //Subscriber to robot status callbacks
    ros::Subscriber m_robotSub;
    std::map<int, ros::ServiceClient > m_statusServices;

    // Set Trash robot service map
    std::map<int, ros::ServiceClient > m_setTrashServices;

    // E-Stop Publisher
    ros::Publisher m_eStopPub;

    // Sound message publisher
    ros::Publisher m_soundPub;

    // Sound message publisher
    ros::Publisher m_textPub;

    // Task Master
    TaskMaster m_tm;

    ros::Time m_lastDisplay;

    boost::mutex m_robotMutex;

    PLANNER_TYPE m_planner;

    // Statistics
    ros::Time m_start_time;

    // Map or robot_id to
    //                    map of waypoint id chosen and seconds since start it was chosen
    std::map<int, std::map<int, double> > robot_waypoint_times;
    //                    map of goal id chosen and seconds since start it was chosen
    std::map<int, std::map<int, double> > robot_goal_times;
    // map<robot_id, map<waypoint_id, double_seconds_since_start> >
};
