#include "ros/ros.h"
#include "RobotStatusWrapper.h"
#include "GoalWrapper.h"
#include "RobotController.h"


class RobotController
{
public:
    RobotController();
    ~RobotController();

    void cb_goalSub(const global_planner::

private:
    // Subscribers to the global planner
    ros::Subscriber m_goalSub;
    ros::Subscriber m_waypointSub;
    ros::Subscriber m_dumpSub;
    ros::Subscriber m_eStopSub;

    ros::Publisher m_statusPub;

    RobotStatusWrapper m_status;

    /**
     * Robot base navigation stuff
     */

    // Nav stack stuff...
    // Move Base members (publishers & subscribers)
    //
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    move_base_msgs::MoveBaseGoal m_goal;

};