#pragma once

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "RobotStatusWrapper.h"
#include "GoalWrapper.h"
#include "WaypointWrapper.h"
#include "DumpWrapper.h"
#include "RobotController.h"

class RobotController
{
public:
    RobotController();
    ~RobotController();

    static const void PoseToMoveBaseGoal(const geometry_msgs::Pose& pose, move_base_msgs::MoveBaseGoal& goal);

    void cb_goalSub(const global_planner::GoalMsg::ConstPtr& msg);
    void cb_waypointSub(const global_planner::WaypointMsg::ConstPtr& msg);
    void cb_dumpSub(const global_planner::DumpMsg::ConstPtr& msg);
    void cb_eStopSub(const std_msgs::Empty& msg);

    // void cb_statusService(const std_msgs::Int32 id);

    //Send the robot's status message
    void SendRobotStatus();

    void SendGoalFinished(TaskResult::Status status);
    void SendWaypointFinished(TaskResult::Status status);
    void SendDumpFinished(TaskResult::Status status);

    void Init(ros::NodeHandle* nh, int robotID = -1, std::string robotName = "", int storage_cap = 3, int storage_used = 0, bool type = true);
    void Execute();

    void Stop();
    void Resume();

private:
    void UpdateStatus();

    // Subscribers to the global planner
    ros::Subscriber m_goalSub;
    ros::Subscriber m_waypointSub;
    ros::Subscriber m_dumpSub;
    ros::Subscriber m_eStopSub;

    ros::Publisher m_statusPub;

    ros::ServiceServer m_statusService;

    RobotStatusWrapper m_status;

    /**
     * Robot base navigation stuff
     */

    // Nav stack stuff...
    // Move Base members (publishers & subscribers)
    //
    // TODO: Remap the needed topics for nav stack
    MoveBaseClient moveBaseClient;
};

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
