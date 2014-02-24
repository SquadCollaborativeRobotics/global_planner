/***********************************************************************
 * AUTHOR: shawn <shawn>
 *   FILE: RobotController.cpp
 *   DATE: Mon Feb 24 10:36:07 2014
 *  DESCR:
 ***********************************************************************/
#include "global_planner/RobotController.h"

/***********************************************************************
 *  Method: RobotController::RobotController
 *  Params:
 * Effects:
 ***********************************************************************/
RobotController::RobotController()
{
}


/***********************************************************************
 *  Method: RobotController::~RobotController
 *  Params:
 * Effects:
 ***********************************************************************/
RobotController::~RobotController()
{
}


/***********************************************************************
 *  Method: RobotController::Init
 *  Params: ros::NodeHandle *nh, int robotID, std::string robotName
 * Returns: void
 * Effects:
 ***********************************************************************/
void RobotController::Init(ros::NodeHandle *nh, int robotID, std::string robotName, int storage_cap, int storage_used, bool type)
{
    m_nh = nh;
    if (robotID < 0)
    {
        ROS_ERROR_STREAM("ERROR: Received invalid robot id: "<<robotID);
        return;
    }
    if (storage_cap < 0)
    {
        ROS_ERROR_STREAM("ERROR: invalid storage capacity: "<<storage_cap);
        return;
    }
    if (storage_used > storage_cap)
    {
        ROS_ERROR_STREAM("ERROR: storage used > storage capacity: used="<<storage_used<< ", cap="<<storage_cap);
        return;
    }

    m_status.SetID(robotID);
    m_status.SetName(robotName);
    // m_status.SetPose(geometry_msgs::Pose pose){ m_status.pose = pose; };
    // m_status.SetTwist(geometry_msgs::Twist twist){ m_status.twist = twist; };
    m_status.SetStorageCapacity(storage_cap);
    m_status.SetStorageUsed(storage_used);
    m_status.SetType(type);

    SetupCallbacks();

    action_client_ptr.reset( new MoveBaseClient("move_base", true) );
}


/***********************************************************************
 *  Method: RobotController::cb_goalSub
 *  Params: const global_planner::GoalMsg::ConstPtr &msg
 * Returns: void
 * Effects:
 ***********************************************************************/
void RobotController::cb_goalSub(const global_planner::GoalMsg::ConstPtr &msg)
{
    GoalWrapper gWrapper;
    global_planner::GoalMsg gm= *msg;
    gWrapper.SetData(gm);

    //Check if this message is for you!
    if (gWrapper.GetRobot() == m_status.GetID())
    {
        ROS_INFO_STREAM("Received Goal for me\n"<<gWrapper.ToString());

        switch(m_status.GetState())
        {
            case RobotState::WAITING:
            ROS_INFO_STREAM("Move to goal");
            break;
            case RobotState::NAVIGATING:
            ROS_ERROR_STREAM("Woah there nelly, we are already navigating to a waypoint... cancel that action first");
            //TODO: Cancel
            break;
            case RobotState::DUMPING:
            ROS_ERROR_STREAM("Get out of here, I'm making a dump... cancel that action first");
            //TODO: Cancel
            break;
            case RobotState::COLLECTING:
            ROS_ERROR_STREAM("Hold on, I'm collecting the shit out of something");
            //TODO: Cancel
            break;
        }
    }
}


/***********************************************************************
 *  Method: RobotController::cb_waypointSub
 *  Params: const global_planner::WaypointMsg::ConstPtr &msg
 * Returns: void
 * Effects:
 ***********************************************************************/
void RobotController::cb_waypointSub(const global_planner::WaypointMsg::ConstPtr &msg)
{
    WaypointWrapper wpWrapper;
    global_planner::WaypointMsg wm= *msg;
    wpWrapper.SetData(wm);

    //Check if this message is for you!
    if (wpWrapper.GetRobot() == m_status.GetID())
    {
        ROS_INFO_STREAM("Received waypoint for me\n"<<wpWrapper.ToString());

        move_base_msgs::MoveBaseGoal goal = Conversion::PoseToMoveBaseGoal(wpWrapper.GetPose());

        switch(m_status.GetState())
        {
            case RobotState::WAITING:
            ROS_INFO_STREAM("Move to new waypoint");
            Transition(RobotState::NAVIGATING, &goal);
            ROS_INFO_STREAM("Sent Waypoint");

            break;
            case RobotState::NAVIGATING:
            ROS_ERROR_STREAM("Woah there nelly, we are already navigating to a waypoint... cancel that action first");
            //TODO: Cancel
            ROS_INFO_STREAM("Now that I assume we've canceled that, let's reassign the waypoint.");
            break;
            case RobotState::DUMPING:
            ROS_ERROR_STREAM("Get out of here, I'm making a dump... cancel that action first");
            //TODO: Cancel
            break;
            case RobotState::COLLECTING:
            ROS_ERROR_STREAM("Hold on, I'm collecting the shit out of something");
            //TODO: Cancel
            break;
        }
    }
}


/***********************************************************************
 *  Method: RobotController::cb_dumpSub
 *  Params: const global_planner::DumpMsg::ConstPtr &msg
 * Returns: void
 * Effects:
 ***********************************************************************/
void RobotController::cb_dumpSub(const global_planner::DumpMsg::ConstPtr &msg)
{
    DumpWrapper dumpWrapper;
    global_planner::DumpMsg dm= *msg;
    dumpWrapper.SetData(dm);

    //Check if this message is for you!
    if (dumpWrapper.GetRobot1() == m_status.GetID() || dumpWrapper.GetRobot2() == m_status.GetID())
    {
        ROS_INFO_STREAM("Received Dump for me\n"<<dumpWrapper.ToString());

        switch(m_status.GetState())
        {
            case RobotState::WAITING:
            ROS_INFO_STREAM("Move to new waypoint");
            break;
            case RobotState::NAVIGATING:
            ROS_ERROR_STREAM("Woah there nelly, we are already navigating to a waypoint... cancel that action first");
            //TODO: Cancel
            ROS_INFO_STREAM("Now that I assume we've canceled that, let's reassign the waypoint.");
            break;
            case RobotState::DUMPING:
            ROS_ERROR_STREAM("Get out of here, I'm making a dump... cancel that action first");
            //TODO: Cancel
            break;
            case RobotState::COLLECTING:
            ROS_ERROR_STREAM("Hold on, I'm collecting the shit out of something");
            //TODO: Cancel
            break;
        }
    }
}


/***********************************************************************
 *  Method: RobotController::cb_eStopSub
 *  Params: const std_msgs::Empty &msg
 * Returns: void
 * Effects:
 ***********************************************************************/
void RobotController::cb_eStopSub(const std_msgs::Empty &msg)
{
}


/***********************************************************************
 *  Method: RobotController::SendRobotStatus
 *  Params:
 * Returns: void
 * Effects:
 ***********************************************************************/
void RobotController::SendRobotStatus()
{
}


/***********************************************************************
 *  Method: RobotController::SendGoalFinished
 *  Params: TaskResult::Status status
 * Returns: void
 * Effects:
 ***********************************************************************/
void RobotController::SendGoalFinished(TaskResult::Status status)
{
}


/***********************************************************************
 *  Method: RobotController::SendWaypointFinished
 *  Params: TaskResult::Status status
 * Returns: void
 * Effects:
 ***********************************************************************/
void RobotController::SendWaypointFinished(TaskResult::Status status)
{
}


/***********************************************************************
 *  Method: RobotController::SendDumpFinished
 *  Params: TaskResult::Status status
 * Returns: void
 * Effects:
 ***********************************************************************/
void RobotController::SendDumpFinished(TaskResult::Status status)
{
}


/***********************************************************************
 *  Method: RobotController::Execute
 *  Params:
 * Returns: void
 * Effects:
 ***********************************************************************/
void RobotController::Execute()
{
    ros::spinOnce();
    ROS_INFO_STREAM_THROTTLE(3, m_status.ToString());

    // 1) Check to see if robot has reached goal & transition if needed
    // 2) Perform any state related actions
    StateExecute();
}


/***********************************************************************
 *  Method: RobotController::Stop
 *  Params:
 * Returns: void
 * Effects:
 ***********************************************************************/
void RobotController::Stop()
{
}


/***********************************************************************
 *  Method: RobotController::Resume
 *  Params:
 * Returns: void
 * Effects:
 ***********************************************************************/
void RobotController::Resume()
{
}


/***********************************************************************
 *  Method: RobotController::UpdateStatus
 *  Params:
 * Returns: void
 * Effects:
 ***********************************************************************/
void RobotController::UpdateStatus()
{
}


/***********************************************************************
 *  Method: RobotController::Finished
 *  Params:
 * Returns: void
 * Effects:
 ***********************************************************************/
void RobotController::Finished()
{
    ROS_INFO_STREAM("FINISHED");
}


/***********************************************************************
 *  Method: RobotController::SetupCallbacks
 *  Params:
 * Returns: void
 * Effects:
 ***********************************************************************/
void RobotController::SetupCallbacks()
{
    m_goalSub = m_nh->subscribe("goal_pub", 10, &RobotController::cb_goalSub, this);
    m_waypointSub = m_nh->subscribe("waypoint_pub", 10, &RobotController::cb_waypointSub, this);
    m_dumpSub = m_nh->subscribe("dump_pub", 10, &RobotController::cb_dumpSub, this);
    m_eStopSub = m_nh->subscribe("e_stop_pub", 10, &RobotController::cb_eStopSub, this);

    ros::Publisher m_statusPub = m_nh->advertise<global_planner::RobotStatus>("robot_status", 100);
}


/***********************************************************************
 *  Method: RobotController::Transition
 *  Params: RobotState::State newState
 * Returns: void
 * Effects:
 ***********************************************************************/
void RobotController::Transition(RobotState::State newState, void* args)
{
    m_status.SetState(newState);
    OnEntry(args);
}


/***********************************************************************
 *  Method: RobotController::OnEntry
 *  Params: RobotState::State state, void *args
 * Returns: void
 * Effects:
 ***********************************************************************/
void RobotController::OnEntry(void *args)
{
    switch(m_status.GetState())
    {
        case RobotState::NAVIGATING:
        ROS_INFO_STREAM("Starting OnEntry: Navigation state");
        move_base_msgs::MoveBaseGoal *goal = (move_base_msgs::MoveBaseGoal *) args;
        action_client_ptr->sendGoal(*goal);
        ROS_INFO_STREAM("Finished OnEntry: Navigation state");
        break;
    }
}


/***********************************************************************
 *  Method: RobotController::StateExecute
 *  Params: RobotState::State state, void *args
 * Returns: void
 * Effects:
 ***********************************************************************/
void RobotController::StateExecute()
{
    // WHILE in WAITING:
    //      IF received a waypoint: transition(NAVIGATING)
    //      IF received a goal message (trash can): transition(COLLECTING)
    //      IF received a dump message: transition(DUMPING)
    // WHILE in COLLECTING:
    //      IF received a waypoint: transition(NAVIGATING)
    //      IF received a dump message: transition(DUMPING)
    //      IF received a stop/cancel/estop: transition(WAITING)
    //      IF reached final pose of the goal, send goal finished message, transition(WAITING)
    // WHILE in WAYPOINT:
    //      IF received a (different) waypoint: change the pose to the new one
    //      IF received a dump message: transition(DUMPING)
    //      IF received a stop/cancel/estop: send waypoint result message (forced_stop) -> transition(WAITING)
    //      IF reached final pose of the waypoint, send waypoint result message (succeed) -> transition(WAITING)
    // ...
    //
    switch(m_status.GetState())
    {
        case RobotState::NAVIGATING:
        if (action_client_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO_STREAM("Successful movebase moving?");
            SendWaypointFinished(TaskResult::SUCCESS);
            Transition(RobotState::WAITING, 0);
        }
        else if (action_client_ptr->getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {
            ROS_INFO_STREAM_THROTTLE(1, "Actively going to goal... NAVIGATING state");
        }
        else
        {
            ROS_INFO_STREAM_THROTTLE(1, "Not yet successful");
        }
    }
}


