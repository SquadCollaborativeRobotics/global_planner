/**
 *  RobotController.h
 *
 *  This class manages the lower level "black box" actions of a robot, including keeping
 *      track of its current state, and transitioning between these states to navigate
 *      and collect garbage from the environment.
 *
 *  (c) 2014
 */

#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Empty.h>

#include <global_planner/GoalFinished.h>
#include <global_planner/WaypointFinished.h>
#include <global_planner/DumpFinished.h>

#include "RobotStatusWrapper.h"
#include "GoalWrapper.h"
#include "WaypointWrapper.h"
#include "DumpWrapper.h"
#include "RobotController.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class RobotController
{
public:
    RobotController();
    ~RobotController();

    void cb_goalSub(const global_planner::GoalMsg::ConstPtr& msg);
    void cb_waypointSub(const global_planner::WaypointMsg::ConstPtr& msg);
    void cb_dumpSub(const global_planner::DumpMsg::ConstPtr& msg);
    void cb_eStopSub(const std_msgs::Empty& msg);
    void cb_odomSub(const nav_msgs::Odometry::ConstPtr& msg);

    // void cb_statusService(const std_msgs::Int32 id);

    //Send the robot's status message
    void SendRobotStatus();

    void SendGoalFinished(TaskResult::Status status);
    void SendWaypointFinished(TaskResult::Status status);
    void SendDumpFinished(TaskResult::Status status);

    void Init(ros::NodeHandle* nh, int robotID = -1, std::string robotName = "", int storage_cap = 3, int storage_used = 0, bool type = true);
    void Execute();
    void Finished();

    void Stop();
    void Resume();

private:
    void SetupCallbacks();
    void UpdatePose();

    void Transition(RobotState::State newState, void* args=0);
    void OnEntry(void* args=0);
    void StateExecute();

    // Subscribers to the global planner
    ros::Subscriber m_goalSub;
    ros::Subscriber m_waypointSub;
    ros::Subscriber m_dumpSub;
    ros::Subscriber m_eStopSub;
    ros::Subscriber m_odomSub;

    ros::Publisher m_statusPub;

    ros::Publisher m_goalFinishedPub;
    ros::Publisher m_waypointFinishedPub;
    ros::Publisher m_dumpFinishedPub;

    // ros::ServiceServer m_statusService;

    RobotStatusWrapper m_status;

    ros::NodeHandle* m_nh;
    tf::TransformListener* m_listener;

    std::string base_frame;
    /**
     * Robot base navigation stuff
     */

    // Nav stack stuff...
    // Move Base members (publishers & subscribers)
    //
    // TODO: Remap the needed topics for nav stack
    boost::shared_ptr<MoveBaseClient> action_client_ptr;

    // Mutex to make sure we are only getting commands when we should (mainly: not during transitions)
    boost::mutex m_statusMutex;

};
