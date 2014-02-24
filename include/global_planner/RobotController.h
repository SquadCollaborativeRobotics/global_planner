#pragma oncee

#include "ros/ros.h"
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

    void cb_goalSub(const global_planner::GoalMsg::ConstPtr& msg);
    void cb_waypointSub(const global_planner::WaypointMsg::ConstPtr& msg);
    void cb_dumpSub(const global_planner::DumpMsg::ConstPtr& msg);
    void cb_eStopSub(const std_msgs::Empty& msg);

    // void cb_statusService(const std_msgs::Int32 id);

    void SendRobotStatus();

    void SendGoalFinished(TaskResult::Status status);
    void SendWaypointFinished(TaskResult::Status status);
    void SendDumpFinished(TaskResult::Status status);

    static const void PoseToMoveBaseGoal(const geometry_msgs::Pose& pose, move_base_msgs::MoveBaseGoal& goal);

private:
    // Subscribers to the global planner
    ros::Subscriber m_goalSub;
    ros::Subscriber m_waypointSub;
    ros::Subscriber m_dumpSub;
    ros::Subscriber m_eStopSub;

    ros::Publisher m_statusPub;

    ros::ServiceServer m_statusService;

    RobotStatusWrapper m_status;

    void UpdateStatus();

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