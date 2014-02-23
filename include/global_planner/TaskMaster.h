#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <boost/shared_ptr.hpp>

#include "RobotStatusWrapper.h"
#include "GoalWrapper.h"
#include "WaypointWrapper.h"
#include "DumpWrapper.h"

#include "global_planner/GoalSeen.h"

#include "global_planner/GoalFinished.h"
#include "global_planner/WaypointFinished.h"
#include "global_planner/DumpFinished.h"

#include "Conversion.h"
#include "TaskResult.h"

class TaskMaster
{
public:

    TaskMaster();
    ~TaskMaster(){};

    // Pass in the NodeHandle, as well as the list of robots on the network (for setting up callbacks/publishers)
    bool Init(ros::NodeHandle& nh, std::map<int, Robot_Ptr > robots);

    // Add goal to the goal list
    bool AddGoal(Goal_Ptr goal);
    // Add goal to the waypoint list
    bool AddWaypoint(Waypoint_Ptr waypoint);
    // Add goal to the dump list
    bool AddDump(Dump_Ptr dump);

    // Retun a pointer to the goal
    Goal_Ptr GetGoal(int goalID);
    // Retun a pointer to the waypoint
    Waypoint_Ptr GetWaypoint(int wpID);
    // Retun a pointer to the dump
    Dump_Ptr GetDump(int dumpID);

    //Assign robot (robotID) to goal (pose)
    bool UpdateGoal(int goalID, int robotID, geometry_msgs::PoseStamped pose);
    //Assign robot (robotID) to waypoint (pose)
    bool UpdateWaypoint(int wpID, int robotID, geometry_msgs::PoseStamped pose);
    //Assign robots (robotID1 & robotID2) to goals (pose1 & pose2)
    bool UpdateDump(int dumpID, int robotID1, int robotID2, geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2);

    // Remove goal
    bool RemoveGoal(int goalID);
    // Remove waypoint
    bool RemoveWaypoint(int wpID);
    // Remove Dump
    bool RemoveDump(int dumpID);

    //Clear all lists of goals, waypoints, and dumps
    bool Clear();

    // Send messages over ROS
    bool SendWaypoint(int wpID);
    bool SendGoal(int goalID);
    bool SendDump(int dumpID);

    std::vector< Goal_Ptr > GetGoalList();
    std::vector< Waypoint_Ptr > GetWaypointList();
    std::vector< Dump_Ptr > GetDumpList();

    // *******************************
    // ROS Callbacks
    // *******************************
    void cb_goalFinished(const global_planner::GoalFinished::ConstPtr& msg);
    void cb_waypointFinished(const global_planner::WaypointFinished::ConstPtr& msg);
    void cb_dumpFinished(const global_planner::DumpFinished::ConstPtr& msg);
    void cb_goalSeen(const global_planner::GoalSeen::ConstPtr& msg);

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

    /**
     * List of publishers & subscribers that will communicate with the robots
     */
    /**
    // List of goal subcribers accessed by the robot id
    std::map<int, ros::Subscriber > m_goalSubs;
    // List of waypoint subscribers accessed by the robot id
    std::map<int, ros::Subscriber > m_waypointSubs;
    // List of dump subscribers accessed by the robot id
    std::map<int, ros::Subscriber > m_dumpSubs;

    // List of goal publishers accessed by the robot id
    std::map<int, ros::Publisher > m_goalPubs;
    // List of waypoint publishers accessed by the robot id
    std::map<int, ros::Publisher > m_waypointPubs;
    // List of dump publishers accessed by the robot id
    std::map<int, ros::Publisher > m_dumpPubs;
    **/

    //For now, we'll just listen to one topic for goals, waypoints, and dump results,
    //  since everything is being published with an ID anyways
    ros::Subscriber m_goalSub;
    ros::Subscriber m_waypointSub;
    ros::Subscriber m_dumpSub;

    // Also, we only need to publish on one topic, since it'll make it easier for robots to listen to.
    ros::Publisher m_goalPub;
    ros::Publisher m_waypointPub;
    ros::Publisher m_dumpPub;

    ros::Subscriber m_goalSeenSub;
    ros::Subscriber m_dumpNeededSub;

    // List of robots in the system
    std::map< int, Robot_Ptr > m_robots;
};
