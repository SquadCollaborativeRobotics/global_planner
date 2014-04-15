/***********************************************************************
 * AUTHOR: shawn <shawn>
 *   FILE: RobotController.cpp
 *   DATE: Mon Feb 24 10:36:07 2014
 *  DESCR: The Robot Controller class implements the functionality of the
 *		   robot
 ***********************************************************************/
#include "global_planner/RobotController.h"

/***********************************************************************
 *  Method: RobotController::RobotController
 *  Params:
 * Effects: Initializer, sets pointers to null
 ***********************************************************************/
RobotController::RobotController():
m_nh(0)
{
    m_status.SetState(RobotState::UNINITIALIZED);
}


/***********************************************************************
 *  Method: RobotController::~RobotController
 *  Params:
 * Effects: Destructor, no need to delete smart pointers
 ***********************************************************************/
RobotController::~RobotController()
{
}


/***********************************************************************
 *  Method: RobotController::Init
 *  Params: ros::NodeHandle *nh, int robotID, std::string robotName, int storage_cap, int storage_used, bool type
 * Returns: void
 * Effects: Initialize the controller with parameters
 ***********************************************************************/
void RobotController::Init(ros::NodeHandle *nh, int robotID, std::string robotName, int storage_cap, int storage_used, RobotState::Type type)
{
    m_nh = nh;
    m_listener.reset( new tf::TransformListener(*nh) );

    if (m_nh->getParam("controller/robot_id", robotID))
        ROS_INFO_STREAM("Set robot ID: " << robotID);
    else
        ROS_ERROR_STREAM("Did not read robot ID: default = " << robotID);
    m_status.SetID(robotID);

    if (m_nh->getParam("controller/robot_name", robotName))
        ROS_INFO_STREAM("Read robot name: " << robotName);
    else
        ROS_ERROR_STREAM("Did not read robot name: default = " << robotName);
    m_status.SetName(robotName);

    if(m_nh->getParam("controller/storage_capacity", storage_cap))
        ROS_INFO_STREAM("Read storage capacity: " << storage_cap);
    else
        ROS_ERROR_STREAM("Did not read storage capacity: default = " << storage_cap);
    m_status.SetStorageCapacity(storage_cap);

    if(m_nh->getParam("controller/storage_used", storage_used))
        ROS_INFO_STREAM("Read storage used: " << storage_used);
    else
        ROS_ERROR_STREAM("Did not read storage used: default = " << storage_used);
    m_status.SetStorageUsed(storage_used);

    std::string robotType;
    if (m_nh->getParam("controller/type", robotType))
        ROS_INFO_STREAM("Read robot type: " << robotType);
    else
        ROS_ERROR_STREAM("Did not read type: default = " << robotType);

    if (robotType.compare("collector") == 0)
    {
        ROS_INFO_STREAM("Type: Collector");
        type = RobotState::COLLECTOR_BOT;
    }
    else
    {
        ROS_INFO_STREAM("Type: Binbot");
        type = RobotState::BIN_BOT;
    }

    m_status.SetType(type);

    std::string tf_prefix;
    if (m_nh->getParam("controller/tf_prefix", tf_prefix))
        ROS_INFO_STREAM("Read tf prefix: " << tf_prefix);
    else
        ROS_ERROR_STREAM("Did not read tf_prefix: default = " << tf_prefix);

    if (m_nh->getParam("controller/base_frame", base_frame))
        ROS_INFO_STREAM("Read base frame: " << base_frame);
    else
        ROS_ERROR_STREAM("Did not read base_frame: default = " << base_frame);

    base_frame = tf_prefix + "/"+base_frame;

    if (robotID < 0)
    {
        ROS_ERROR_STREAM("ERROR: Received invalid robot id: " << robotID);
        return;
    }
    if (storage_cap < 0)
    {
        ROS_ERROR_STREAM("ERROR: invalid storage capacity: " << storage_cap);
        return;
    }
    if (storage_used > storage_cap)
    {
        ROS_ERROR_STREAM("ERROR: storage used > storage capacity: used=" << storage_used <<  ", cap=" << storage_cap);
        return;
    }

    m_tagProcessor.reset( new AprilTagProcessor() );
    m_tagProcessor->Init(nh, robotID);

    action_client_ptr.reset( new MoveBaseClient("move_base", true) );
    // Wait for the action server to come up
    while(ros::ok() && !action_client_ptr->waitForServer(ros::Duration(1.0))){
        ROS_INFO_THROTTLE(3.0,"Waiting for the move_base action server to come up");
    }


    SetupCallbacks();

    ROS_DEBUG_STREAM("Robot has setup the movebase client");

    Transition(RobotState::WAITING);
    m_timeEnteringState = ros::Time::now();

    ROS_INFO_STREAM("Finished initializing");
}


/***********************************************************************
 *  Method: RobotController::cb_goalSub
 *  Params: const global_planner::GoalMsg &msg
 * Returns: void
 * Effects: callback for goal messages
 ***********************************************************************/
bool RobotController::cb_goalSub(global_planner::GoalSrv::Request  &req,
                                 global_planner::GoalSrv::Response &res)
{
    GoalWrapper goalWrapper;
    global_planner::GoalMsg goalMsg = req.msg;
    goalWrapper.SetData(goalMsg);
    res.result = -1;

    //Check if this message is for you!
    if (goalWrapper.GetRobot() == m_status.GetID())
    {
        ROS_DEBUG_STREAM("Received goal for me:\n"<<goalWrapper.ToString());

        switch(m_status.GetState())
        {
            case RobotState::WAITING:
                m_moveBaseGoal = Conversion::PoseToMoveBaseGoal(goalWrapper.GetPose());
                ROS_INFO_STREAM("Received command to go to goal ("<<goalWrapper.GetID()<<")");
                m_status.SetTaskID( goalWrapper.GetID() );
                Transition(RobotState::COLLECTING);
                res.result = 0;
            break;
            case RobotState::NAVIGATING:
                ROS_ERROR_STREAM("Currently navigating to waypoint, canceling waypoint.");
                SendWaypointFinished(TaskResult::FORCE_STOP);

                m_moveBaseGoal = Conversion::PoseToMoveBaseGoal(goalWrapper.GetPose());
                ROS_INFO_STREAM("Received command to go to goal ("<<goalWrapper.GetID()<<")");
                m_status.SetTaskID( goalWrapper.GetID() );
                Transition(RobotState::COLLECTING);
                res.result = 0;
                ROS_INFO_STREAM("Now that we've canceled that, let's reassign the robot to move to the goal.");
            break;
            case RobotState::DUMPING:
                ROS_ERROR_STREAM("Currently navigating to dump, ignoring goal.");
                res.result = -1;
                //TODO: Cancel
            break;
            case RobotState::COLLECTING:
                ROS_ERROR_STREAM("Currently collecting, ignoring goal.");
                res.result = -1;
                //TODO: Cancel
            break;
            default:
                ROS_ERROR_STREAM("Goal hit: Robot is in state: " << RobotState::ToString(m_status.GetState()) << ", which should not be sent a goal message");
            break;

        }
    }
    return true;
}


/***********************************************************************
 *  Method: RobotController::cb_waypointSub
 *  Params: global_planner::WaypointSrv::Request &req, global_planner::WaypointSrv::Response &res
 * Returns: void
 * Effects: callback for waypoint messages
 ***********************************************************************/
bool RobotController::cb_waypointSub(global_planner::WaypointSrv::Request  &req,
                                     global_planner::WaypointSrv::Response &res)
{
    WaypointWrapper wpWrapper;
    global_planner::WaypointMsg wm= req.msg;
    wpWrapper.SetData(wm);
	// return -1 if not in valid state when this message is received
    res.result = -1;

    //Check if this message is for you!
    if (wpWrapper.GetRobot() == m_status.GetID())
    {
        ROS_DEBUG_STREAM("Received waypoint for me:\n"<<wpWrapper.ToString());

        switch(m_status.GetState())
        {
            case RobotState::DUMPING_FINISHED:
            case RobotState::WAITING:
                m_moveBaseGoal = Conversion::PoseToMoveBaseGoal(wpWrapper.GetPose());
                ROS_INFO_STREAM("Move to new waypoint ("<<wpWrapper.GetID()<<")");
                m_status.SetTaskID( wpWrapper.GetID() );
                Transition(RobotState::NAVIGATING);
                res.result = 0;
            break;
            default:
                ROS_ERROR_STREAM("Waypoint hit: Robot is in state: "<<RobotState::ToString(m_status.GetState())<<", which should not be sent a waypoint message");
            break;

            /*
            case RobotState::NAVIGATING:
            ROS_ERROR_STREAM("WARNING: In Navigation State, Ignoring Waypoint.");
            break;

            case RobotState::DUMPING:
            ROS_ERROR_STREAM("WARNING: In Dumping State, Ignoring Waypoint.");
            break;

            case RobotState::COLLECTING:
            ROS_ERROR_STREAM("WARNING: In Collecting State, Ignoring Waypoint.");
            break;
            */
        }
    }
}


/***********************************************************************
 *  Method: RobotController::cb_dumpSub
 *  Params: const global_planner::DumpMsg &msg
 * Returns: void
 * Effects: callback for dump messages
 ***********************************************************************/
bool RobotController::cb_dumpSub(global_planner::DumpSrv::Request  &req,
                                 global_planner::DumpSrv::Response &res)
{
    ROS_INFO("::::::::::::::: Received dump");
    DumpWrapper dumpWrapper;
    global_planner::DumpMsg dumpMsg = req.msg;
    dumpWrapper.SetData(dumpMsg);
    res.result = -1;
    // id = -1 if not a match, 1 if robot1 and 2 if robot 2
    int id = dumpWrapper.GetRobot1() == m_status.GetID() ? 1 : dumpWrapper.GetRobot2() == m_status.GetID() ? 2 : 0;

    //Check if this message is for you!
    if (id)
    {
        ROS_DEBUG_STREAM("Received dump for me:\n"<<dumpWrapper.ToString());

        switch(m_status.GetState())
        {
            case RobotState::NAVIGATING:
                ROS_ERROR_STREAM("Currently navigating to waypoint, canceling waypoint.");
                SendWaypointFinished(TaskResult::FORCE_STOP);
                // continue on and do add new task also
            case RobotState::WAITING:
                m_moveBaseGoal = Conversion::PoseToMoveBaseGoal(id == 1 ? dumpWrapper.GetPose1() : dumpWrapper.GetPose2());
                ROS_INFO_STREAM("Received command to go to dump ("<<dumpWrapper.GetID()<<")");
                m_status.SetTaskID( dumpWrapper.GetID() );
                Transition(RobotState::DUMPING);
                res.result = 0;
            break;
            case RobotState::DUMPING:
                ROS_ERROR_STREAM("Currently navigating to dump, ignoring given new dump.");
                res.result = -1;
                //TODO: Cancel
            break;
            case RobotState::COLLECTING:
                ROS_ERROR_STREAM("Currently collecting, ignoring dump.");
                res.result = -1;
                //TODO: Cancel
            break;
            default:
                ROS_ERROR_STREAM("Dump hit: Robot is in state: " << RobotState::ToString(m_status.GetState()) <<", which should not be sent a goal message");
            break;

        }
    }
    return true;
}


/***********************************************************************
 *  Method: RobotController::cb_eStopSub
 *  Params: const std_msgs::Empty &msg
 * Returns: void
 * Effects: emergency stop callback.  it should cancel current goals and
 * 	go to waiting state
 ***********************************************************************/
void RobotController::cb_eStopSub(const std_msgs::Empty &msg)
{
    ROS_INFO("eStop received");
    action_client_ptr->cancelAllGoals();
    Transition(RobotState::ESTOP);
}


/***********************************************************************
 *  Method: RobotController::SendRobotStatus
 *  Params:
 * Returns: void
 * Effects: Sends the current status over ROS to any listening
 *  nodes
 ***********************************************************************/
bool RobotController::SendRobotStatus(global_planner::RobotStatusSrv::Request  &req,
                                      global_planner::RobotStatusSrv::Response &res)
{
    UpdatePose();
    res.status = m_status.GetMessage();
    m_lastStatusUpdate = ros::Time::now();
    return true;
}


/***********************************************************************
 *  Method: RobotController::SendRobotStatus
 *  Params:
 * Returns: void
 * Effects: Sends the current status over ROS to any listening
 *  nodes
 ***********************************************************************/
bool RobotController::cb_SetRobotStatus(global_planner::SetRobotStatusSrv::Request  &req,
                                      global_planner::SetRobotStatusSrv::Response &res)
{
    // res.status = m_status.GetMessage();
    // m_lastStatusUpdate = ros::Time::now();
    // TODO: this
    return false;
}


/***********************************************************************
 *  Method: RobotController::SendGoalFinished
 *  Params: TaskResult::Status status
 * Returns: void
 * Effects: Called when a goal task is finished.  Publish the result
 ***********************************************************************/
void RobotController::SendGoalFinished(TaskResult::Status status)
{
    global_planner::GoalFinished goalMsg;
    goalMsg.id = m_status.GetTaskID();
    goalMsg.status = Conversion::TaskResultToInt(status);

    if (status == TaskResult::SUCCESS)
        m_status.IncrementStorageUsed();

    m_goalFinishedPub.publish(goalMsg);
}


/***********************************************************************
 *  Method: RobotController::SendWaypointFinished
 *  Params: TaskResult::Status status
 * Returns: void
 * Effects: Called when a waypoint task is finished.  Publish the result
 ***********************************************************************/
void RobotController::SendWaypointFinished(TaskResult::Status status)
{
    global_planner::WaypointFinished wpMsg;
    wpMsg.id = m_status.GetTaskID();
    wpMsg.status = Conversion::TaskResultToInt(status);

    m_waypointFinishedPub.publish(wpMsg);
}


/***********************************************************************
 *  Method: RobotController::SendDumpFinished
 *  Params: TaskResult::Status status
 * Returns: void
 * Effects: Called when the bot has reached its goal position for dumping
 ***********************************************************************/
void RobotController::SendDumpFinished(TaskResult::Status status)
{
    global_planner::DumpFinished dumpMsg;
    dumpMsg.id = m_status.GetTaskID();
    dumpMsg.robotID = m_status.GetID();
    dumpMsg.status = Conversion::TaskResultToInt(status);

    m_dumpFinishedPub.publish(dumpMsg);
}


/***********************************************************************
 *  Method: RobotController::Execute
 *  Params:
 * Returns: void
 * Effects: Run the controller, specifically running anything that should
 * 	happen when in a state
 ***********************************************************************/
void RobotController::Execute()
{
    ros::spinOnce();
    // ROS_INFO_STREAM_THROTTLE(3, m_status.ToString());

    // 1) Check to see if robot has reached goal & transition if needed
    // 2) Perform any state related actions
    StateExecute();

    //Don't need anymore since we're using services
    if (ros::Time::now() - m_lastStatusUpdate > ros::Duration(5))
    {
        ROS_WARN_STREAM_THROTTLE(10, "Robot has not been in communication for "<<(ros::Time::now() - m_lastStatusUpdate) << " seconds");
        UpdatePose();
        m_statusPub.publish(m_status.GetMessage());
    }
}


/***********************************************************************
 *  Method: RobotController::Stop
 *  Params:
 * Returns: void
 * Effects: TODO: 'pause' function. does whatever it needs for it
 ***********************************************************************/
void RobotController::Stop()
{
}


/***********************************************************************
 *  Method: RobotController::Resume
 *  Params:
 * Returns: void
 * Effects: TODO: 'resume' function, resume robot to previous task
 ***********************************************************************/
void RobotController::Resume()
{
}


/***********************************************************************
 *  Method: RobotController::UpdateStatus
 *  Params:
 * Returns: void
 * Effects: Gathers any needed information to update the underlying
 * 	status message
 ***********************************************************************/
bool RobotController::UpdatePose()
{
    //get current pose of the robot
    tf::StampedTransform transform;
    try{
        m_listener->lookupTransform("/map", base_frame.c_str(),
                      ros::Time(0), transform);

        tf::Transform trans = transform;

        geometry_msgs::Transform msg;
        tf::transformTFToMsg(trans, msg);

        geometry_msgs::Pose pose;
        pose.position.x = msg.translation.x;
        pose.position.y = msg.translation.y;
        pose.orientation.z = msg.rotation.z;
        pose.orientation.w = msg.rotation.w;
        m_status.SetPose(pose);
        return true;
    }
    catch (tf::TransformException& ex){
        ROS_ERROR_THROTTLE(5, "%s",ex.what());
        return false;
    }
}


/***********************************************************************
 *  Method: RobotController::Finished
 *  Params:
 * Returns: void
 * Effects:	TODO: code to run when entire system is finished
 ***********************************************************************/
void RobotController::Finished()
{
    ROS_INFO_STREAM("FINISHED");
}


/***********************************************************************
 *  Method: RobotController::SetupCallbacks
 *  Params:
 * Returns: void
 * Effects:	Sets up the subscribers, publishers, and services for the bot
 ***********************************************************************/
void RobotController::SetupCallbacks()
{
    m_eStopSub = m_nh->subscribe("/e_stop", 10, &RobotController::cb_eStopSub, this);

    m_odomSub = m_nh->subscribe("odom", 10, &RobotController::cb_odomSub, this);

    m_statusPub = m_nh->advertise<global_planner::RobotStatus>("/robot_status", 100);

    m_goalFinishedPub = m_nh->advertise<global_planner::GoalFinished>("/goal_finished", 10);
    m_waypointFinishedPub = m_nh->advertise<global_planner::WaypointFinished>("/waypoint_finished", 10);
    m_dumpFinishedPub = m_nh->advertise<global_planner::DumpFinished>("/dump_finished", 10);

    std::string statusServiceTopic = Conversion::RobotIDToServiceName(m_status.GetID());
    m_statusService = m_nh->advertiseService(statusServiceTopic, &RobotController::SendRobotStatus, this);

    std::string waypointServiceTopic = Conversion::RobotIDToWaypointTopic(m_status.GetID());
    m_waypointService = m_nh->advertiseService(waypointServiceTopic, &RobotController::cb_waypointSub, this);

    std::string goalServiceTopic = Conversion::RobotIDToGoalTopic(m_status.GetID());
    m_goalService = m_nh->advertiseService(goalServiceTopic, &RobotController::cb_goalSub, this);

    std::string dumpServiceTopic = Conversion::RobotIDToDumpTopic(m_status.GetID());
    m_dumpService = m_nh->advertiseService(dumpServiceTopic, &RobotController::cb_dumpSub, this);

    //Let the global planner know that this robot is alive an active
    UpdatePose();
    m_statusPub.publish(m_status.GetMessage());

    ROS_INFO_STREAM("Finished setting up topics");
}


/***********************************************************************
 *  Method: RobotController::Transition
 *  Params: RobotState::State newState
 * Returns: void
 * Effects:	Transition strarts the transition from one state to another
 ***********************************************************************/
void RobotController::Transition(RobotState::State newState, void* args)
{
    //TODO: make sure the state we are entering is valid based on the current state of the robot
    if (m_status.GetState() != newState)
    {
        m_timeEnteringState = ros::Time::now();
    }
    m_status.SetState(newState);
    OnEntry(args);
    ROS_INFO_STREAM("Transitioned to " << RobotState::ToString(newState));
}


/***********************************************************************
 *  Method: RobotController::OnEntry
 *  Params: void *args
 * Returns: void
 * Effects: This is what runs when entering a state
 ***********************************************************************/
void RobotController::OnEntry(void *args)
{
    switch(m_status.GetState())
    {
        case RobotState::WAITING:
            m_status.SetTaskID(-1);
            break;
        case RobotState::NAVIGATING:
            action_client_ptr->sendGoal(m_moveBaseGoal);
            break;
        case RobotState::NAVIGATING_TAG_SPOTTED:
            ROS_INFO_STREAM("Cancelling goals due to april tag processor");
            action_client_ptr->cancelAllGoals();
            break;
        case RobotState::COLLECTING:
            action_client_ptr->sendGoal(m_moveBaseGoal);
            break;
        case RobotState::COLLECTING_TAG_SPOTTED:
            ROS_INFO_STREAM("Cancelling goals due to april tag processor");
            action_client_ptr->cancelAllGoals();
            break;
        case RobotState::DUMPING:
            ROS_INFO_STREAM("Starting navigation to dump site.");
            action_client_ptr->sendGoal(m_moveBaseGoal);
            break;
        case RobotState::DUMPING_FINISHED:
            ROS_INFO_STREAM("Reached Dump Site");
            break;
        break;
    }
}


/***********************************************************************
 *  Method: RobotController::StateExecute
 *  Params: void *args
 * Returns: void
 * Effects: specifies what to run while in a state, and transition if
 *  appropriate
 ***********************************************************************/
void RobotController::StateExecute()
{
    // WHILE in WAITING:
    //      IF received a waypoint: transition(NAVIGATING)
    //      IF received a goal message (trash can): transition(COLLECTING)
    //      IF received a dump message: transition(DUMPING)
    // WHILE in COLLECTING (goal collecting):
    //      IF received a waypoint: transition(NAVIGATING)
    //      IF received a dump message: transition(DUMPING)
    //      IF received a stop/cancel/estop: transition(WAITING)
    //      IF reached final pose of the goal, send goal finished message, transition(WAITING)
    // WHILE in NAVIGATING (waypoint navigation):
    //      IF received a (different) waypoint: change the pose to the new one
    //      IF received a dump message: transition(DUMPING)
    //      IF received a stop/cancel/estop: send waypoint result message (forced_stop) -> transition(WAITING)
    //      IF reached final pose of the waypoint, send waypoint result message (succeed) -> transition(WAITING)
    // WHILE in DUMPING (going to dump):
    //      IF received a (different) waypoint: change the pose to the new one
    //      IF received a dump message: transition(DUMPING)
    //      IF received a stop/cancel/estop: send waypoint result message (forced_stop) -> transition(WAITING)
    //      IF reached final pose of the waypoint, send waypoint result message (succeed) -> transition(WAITING)
    // ...
    bool execResult = false;
    // ROS_INFO_STREAM_THROTTLE(1.0, "STATE: "<<RobotState::ToString(m_status.GetState()));
    switch(m_status.GetState())
    {
        case RobotState::WAITING:

            if (m_tagProcessor->ShouldPause())
            {
                Transition(RobotState::WAITING_TAG_SPOTTED);
            }
        break;

        //In this state, the robot should now be stopping and getting a more accurate view of the tag
        case RobotState::WAITING_TAG_SPOTTED:
            sleep(1.0);
            ros::spinOnce();
            execResult = m_tagProcessor->Execute();
            // ROS_DEBUG_STREAM_THROTTLE(0.5, "Result from execute: "<<execResult);
            if (m_tagProcessor->ShouldResume())
            {
                //Transition back to the navigating state, using the same goal as before
                ROS_INFO("Resuming robot");
                Transition(RobotState::WAITING_TAG_FINISHED);
                break;
            }
            else
            {
                ROS_INFO_STREAM_THROTTLE(1, "Waiting on the OK to resume from the tag processor");
            }
        break;

        case RobotState::WAITING_TAG_FINISHED:
            if (ros::Time::now() - m_timeEnteringState > ros::Duration(2.0))
            {
                Transition(RobotState::WAITING);
            }
            break;

        case RobotState::NAVIGATING:
            if (m_tagProcessor->ShouldPause())
            {
                Transition(RobotState::NAVIGATING_TAG_SPOTTED);
                break;
            }
            else
            {
                actionlib::SimpleClientGoalState::StateEnum result = action_client_ptr->getState().state_;
                switch (result)
                {
                    case actionlib::SimpleClientGoalState::SUCCEEDED:
                        ROS_INFO_STREAM("Movebase reached target.");
                        SendWaypointFinished(TaskResult::SUCCESS);
                        Transition(RobotState::WAITING);
                        break;
                    case actionlib::SimpleClientGoalState::ABORTED:
                    case actionlib::SimpleClientGoalState::REJECTED:
                    case actionlib::SimpleClientGoalState::LOST:
                    case actionlib::SimpleClientGoalState::RECALLED:
                    case actionlib::SimpleClientGoalState::PREEMPTED:
                        ROS_ERROR_STREAM("Navigation Failed: " << action_client_ptr->getState().toString() );
                        SendWaypointFinished(TaskResult::FAILURE);
                        Transition(RobotState::WAITING);
                        break;

                    case actionlib::SimpleClientGoalState::ACTIVE:
                    case actionlib::SimpleClientGoalState::PENDING:
                    default:
                        ROS_INFO_STREAM_THROTTLE(10, "Navigation state = " << action_client_ptr->getState().toString() );
                        break;
                }
            }
        break;

        //In this state, the robot should now be stopping and getting a more accurate view of the tag
        case RobotState::NAVIGATING_TAG_SPOTTED:
            sleep(1.0);
            ros::spinOnce();
            execResult = m_tagProcessor->Execute();
            // ROS_DEBUG_STREAM_THROTTLE(0.5, "Result from execute: "<<execResult);
            if (m_tagProcessor->ShouldResume())
            {
                //Transition back to the navigating state, using the same goal as before
                ROS_INFO("Resuming robot");
                Transition(RobotState::NAVIGATING_TAG_FINISHED);
                break;
            }
            else
            {
                ROS_INFO_STREAM_THROTTLE(1, "Waiting on the OK to resume from the tag processor");
            }
        break;

        case RobotState::NAVIGATING_TAG_FINISHED:
            if (ros::Time::now() - m_timeEnteringState > ros::Duration(2.0))
            {
                Transition(RobotState::NAVIGATING);
            }
            break;

        case RobotState::COLLECTING:
            if (m_tagProcessor->ShouldPause())
            {
                Transition(RobotState::COLLECTING_TAG_SPOTTED);
                break;
            }
            else
            {
                actionlib::SimpleClientGoalState::StateEnum result = action_client_ptr->getState().state_;
                switch (result)
                {
                    case actionlib::SimpleClientGoalState::SUCCEEDED:
                        ROS_INFO_STREAM("Movebase reached target.");
                        SendGoalFinished(TaskResult::SUCCESS);
                        Transition(RobotState::WAITING);
                        break;
                    case actionlib::SimpleClientGoalState::ABORTED:
                    case actionlib::SimpleClientGoalState::REJECTED:
                    case actionlib::SimpleClientGoalState::LOST:
                    case actionlib::SimpleClientGoalState::RECALLED:
                    case actionlib::SimpleClientGoalState::PREEMPTED:
                        ROS_ERROR_STREAM("Collecting Failed: " << action_client_ptr->getState().toString() );
                        SendGoalFinished(TaskResult::FAILURE);
                        Transition(RobotState::WAITING);
                        break;

                    case actionlib::SimpleClientGoalState::ACTIVE:
                    case actionlib::SimpleClientGoalState::PENDING:
                    default:
                        ROS_INFO_STREAM_THROTTLE(10, "Collection state = " << action_client_ptr->getState().toString() );
                        break;
                }
            }
        break;

        //In this state, the robot should now be stopping and getting a more accurate view of the tag
        case RobotState::COLLECTING_TAG_SPOTTED:
            sleep(1.0);
            ros::spinOnce();
            execResult = m_tagProcessor->Execute();
            // ROS_DEBUG_STREAM_THROTTLE(0.5, "Result from execute: "<<execResult);
            if (m_tagProcessor->ShouldResume())
            {
                //Transition back to the COLLECTING state, using the same goal as before
                ROS_INFO("Resuming robot");
                Transition(RobotState::COLLECTING_TAG_FINISHED);
                break;
            }
            else
            {
                ROS_INFO_STREAM_THROTTLE(0.5, "Waiting on the OK to resume from the tag processor");
            }
        break;

        case RobotState::COLLECTING_TAG_FINISHED:
            if (ros::Time::now() - m_timeEnteringState > ros::Duration(2.0))
            {
                Transition(RobotState::COLLECTING);
            }
            break;

        case RobotState::DUMPING:
        {
            // Check if reached dump site yet
            actionlib::SimpleClientGoalState::StateEnum result = action_client_ptr->getState().state_;
            switch (result)
            {
                case actionlib::SimpleClientGoalState::SUCCEEDED:
                    ROS_INFO_STREAM("Movebase reached target.");
                    SendDumpFinished(TaskResult::SUCCESS);
                    Transition(RobotState::DUMPING_FINISHED);
                    break;
                case actionlib::SimpleClientGoalState::ABORTED:
                case actionlib::SimpleClientGoalState::REJECTED:
                case actionlib::SimpleClientGoalState::LOST:
                case actionlib::SimpleClientGoalState::RECALLED:
                case actionlib::SimpleClientGoalState::PREEMPTED:
                    ROS_ERROR_STREAM("Navigation Failed: " << action_client_ptr->getState().toString() );
                    SendDumpFinished(TaskResult::FAILURE);
                    Transition(RobotState::WAITING);
                    break;

                case actionlib::SimpleClientGoalState::ACTIVE:
                case actionlib::SimpleClientGoalState::PENDING:
                default:
                    ROS_INFO_STREAM_THROTTLE(10, "Navigation state = " << action_client_ptr->getState().toString() );
                    break;
            }
            break;
        }

        case RobotState::DUMPING_FINISHED:
        break;

        default:
            ROS_ERROR_STREAM_THROTTLE(5, "Unexpected State:" << RobotState::ToString(m_status.GetState()) );
            break;
    }
}


/***********************************************************************
 *  Method: RobotController::cb_odomSub
 *  Params: const nav_msgs::Odometry::ConstPtr &msg
 * Returns: void
 * Effects: callback for the odometry message for the bot
 ***********************************************************************/
void RobotController::cb_odomSub(const nav_msgs::Odometry::ConstPtr &msg)
{
    m_status.SetTwist(msg->twist.twist);
}


