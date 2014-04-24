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

    base_frame = std::string("base_link");
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
    // Todo: Only start if fake trash is not used
    m_tagProcessor.reset( new AprilTagProcessor() );
    m_tagProcessor->Init(nh, robotID);

    action_client_ptr.reset( new MoveBaseClient("move_base", true) );
    // Wait for the action server to come up
    while(ros::ok() && !action_client_ptr->waitForServer(ros::Duration(2.0))){
        ROS_INFO_THROTTLE(3.0,"Waiting for the move_base action server to come up");
    }


    SetupCallbacks();

    ROS_DEBUG_STREAM("Robot has setup the movebase client");

    Transition(RobotState::WAITING);
    m_timeEnteringState = ros::Time::now();

    ROS_INFO_STREAM("Finished initializing");
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
            case RobotState::WAITING_TAG_SPOTTED:
            case RobotState::NAVIGATING_TAG_SPOTTED:
                res.result = -1;
                ROS_ERROR_STREAM("Currently processing april tag... do not accept waypoint");
            break;
            default:
                ROS_ERROR_STREAM("Waypoint hit: Robot is in state: "<<RobotState::ToString(m_status.GetState())<<", which should not be sent a waypoint message");
                res.result = -1;
            break;
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
    // id = 0 if not a match, 1 if robot1 and 2 if robot 2
    int id = dumpWrapper.GetRobot1() == m_status.GetID() ? 1 : dumpWrapper.GetRobot2() == m_status.GetID() ? 2 : 0;

    //Check if this message is for you!
    if (id > 0)
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
            break;
            default:
                ROS_ERROR_STREAM("Dump hit: Robot is in state: " << RobotState::ToString(m_status.GetState()) <<", which should not be sent a dump message");
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
    if (req.status.storage_used >= 0)
    {
        m_status.SetStorageUsed(req.status.storage_used);
    }
    if (req.status.storage_capacity >= 0)
    {
        m_status.SetStorageCapacity(req.status.storage_capacity);
    }
    if (req.status.taskID >= 0)
    {
        m_status.SetTaskID(req.status.taskID);
    }

    res.res = 0;

    return true;
}


/***********************************************************************
 *  Method: RobotController::cb_SetTrash
 *  Params:
 * Returns: void
 * Effects: Called to set the amount of storage used by robot
 ***********************************************************************/
bool RobotController::cb_SetTrash(global_planner::SetTrashSrv::Request  &req,
                                  global_planner::SetTrashSrv::Response &res)
{
    // If robot can hold requested trash, set it.
    if (m_status.GetStorageCapacity() >= req.storage)
    {
        m_status.SetStorageUsed(req.storage); // Set storage to requested storage

        // Horrible Hack due to service issues, need to have a binbot transition to waiting state once trash is transferred
        if (m_status.GetState() == RobotState::DUMPING_FINISHED && m_status.GetType() == RobotState::BIN_BOT)
        {
            ROS_INFO("TEMPORARY : Transition from DUMPING_FINISHED to WAITING due to trash loading");
            Transition(RobotState::WAITING);
        }

        res.result = 0; // Success
    }
    else
    {
        res.result = -1; // Failure
        return false;
    }
    return true;
}


/***********************************************************************
 *  Method: RobotController::SendWaypointFinished
 *  Params: TaskResult::Status status
 * Returns: void
 * Effects: Called when a waypoint task is finished.  Publish the result
 ***********************************************************************/
bool RobotController::SendWaypointFinished(TaskResult::Status status)
{
    if (m_status.GetTaskID() < 0)
    {
        ROS_INFO_STREAM("m_status task id invalid: "<<m_status.GetTaskID());
        return false;
    }

    global_planner::WaypointFinished wpMsg;
    wpMsg.request.id = m_status.GetTaskID();
    wpMsg.request.status = Conversion::TaskResultToInt(status);

    if (!m_waypointFinishedPub.isValid())
    {
        m_waypointFinishedPub = m_nh->serviceClient<global_planner::WaypointFinished>(Conversion::RobotIDToWaypointFinishedTopic(m_status.GetID()), true);
    }

    if (m_waypointFinishedPub.isValid())
    {
        if (m_waypointFinishedPub.call(wpMsg))
        {
            ROS_INFO_STREAM("Sending waypoint finished success");
            return true;
        }
        else
        {
            ROS_ERROR_STREAM("Failed to call waypoint finished service");
            return false;
        }
    }
    else
    {
        ROS_ERROR_STREAM("Still not connected to waypoint finished service");
        return false;
    }
    return false;
}


/***********************************************************************
 *  Method: RobotController::SendDumpFinished
 *  Params: TaskResult::Status status
 * Returns: void
 * Effects: Called when the bot has reached its goal position for dumping
 ***********************************************************************/
bool RobotController::SendDumpFinished(TaskResult::Status status)
{
    if (m_status.GetTaskID() < 0)
        return false;

    global_planner::DumpFinished dumpMsg;
    dumpMsg.request.id = m_status.GetTaskID();
    dumpMsg.request.robotID = m_status.GetID();
    dumpMsg.request.status = Conversion::TaskResultToInt(status);

    if (m_dumpFinishedPub)
    {
        if (m_dumpFinishedPub.call(dumpMsg))
        {
            ROS_INFO_STREAM("Sending dump finished success");
        }
        else
        {
            ROS_ERROR_STREAM("Failed to call dump finished service");
        }
        ROS_ERROR_STREAM("dump finished service not setup properly");
    }
    return true;
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
    if (ros::Time::now() - m_lastStatusUpdate > ros::Duration(2))
    {
        ROS_WARN_STREAM_THROTTLE(10, "Robot has not been in communication for "<<(ros::Time::now() - m_lastStatusUpdate) << " seconds");
        UpdatePose();
        m_statusPub.publish(m_status.GetMessage());
        // m_lastStatusUpdate = ros::Time::now();
    }

    // Constant update every 200ms ~ 5 Hz
    if (ros::Time::now() - m_lastConstantStatusUpdate > ros::Duration(0.2))
    {
        UpdatePose();
        m_statusPub.publish(m_status.GetMessage());
        m_lastConstantStatusUpdate = ros::Time::now();
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

    ROS_INFO_STREAM("Connecting to service: "<<Conversion::RobotIDToWaypointFinishedTopic(m_status.GetID()));
    // ROS_INFO_STREAM("Waiting for wp service to come up...");
    // ros::service::waitForService( Conversion::RobotIDToWaypointFinishedTopic(m_status.GetID()) );
    //Task Finished services
    m_waypointFinishedPub = m_nh->serviceClient<global_planner::WaypointFinished>(Conversion::RobotIDToWaypointFinishedTopic(m_status.GetID()), false);
    if (m_waypointFinishedPub.isValid())
    {
        ROS_INFO_STREAM("Successfully connected to wp finished service");
    }
    else
    {
        ROS_ERROR_STREAM("Could not connect to wp finished service.");
    }

    ROS_INFO_STREAM("Connecting to service: "<<Conversion::RobotIDToDumpFinishedTopic(m_status.GetID()));
    // ROS_INFO_STREAM("Waiting for dump service to come up...");
    // ros::service::waitForService( Conversion::RobotIDToDumpFinishedTopic(m_status.GetID()) );
    m_dumpFinishedPub = m_nh->serviceClient<global_planner::DumpFinished>(Conversion::RobotIDToDumpFinishedTopic(m_status.GetID()), true);
    if (m_dumpFinishedPub.isValid())
    {
        ROS_INFO_STREAM("Successfully connected to dump finished service");
    }
    else
    {
        ROS_ERROR_STREAM("Could not connect to dump finished service.");
    }

    m_statusService = m_nh->advertiseService(Conversion::RobotIDToServiceName(m_status.GetID()), &RobotController::SendRobotStatus, this);
    m_waypointService = m_nh->advertiseService(Conversion::RobotIDToWaypointTopic(m_status.GetID()), &RobotController::cb_waypointSub, this);
    m_dumpService = m_nh->advertiseService(Conversion::RobotIDToDumpTopic(m_status.GetID()), &RobotController::cb_dumpSub, this);
    m_setStatusService = m_nh->advertiseService(Conversion::RobotIDToSetStatusTopic(m_status.GetID()), &RobotController::cb_SetRobotStatus, this);
    m_setTrashService = m_nh->advertiseService(Conversion::RobotIDToSetTrash(m_status.GetID()), &RobotController::cb_SetTrash, this);

    ros::spinOnce();

    // Keep trying to connect to the global planner until we see the waypoint finished service running
    m_status.SetState(RobotState::UNINITIALIZED);
    bool waypointFin = dumpFin = false;
    while (waypointFin == false || dumpFin == false)
    {
        //Task Finished services
        ROS_INFO_STREAM("Connecting to service: "<<Conversion::RobotIDToWaypointFinishedTopic(m_status.GetID()));
        ROS_INFO_STREAM("Waiting up to 10 seconds for waypoint finished service to come up");
        // Setup waypoint finished service client
        while (true)
        {
            bool success = ros::service::waitForService(Conversion::RobotIDToWaypointFinishedTopic(m_status.GetID()), ros::Duration(5));
            if (success)
            {
                m_waypointFinishedPub = m_nh->serviceClient<global_planner::WaypointFinished>(Conversion::RobotIDToWaypointFinishedTopic(m_status.GetID()), false);
                if (m_waypointFinishedPub.isValid())
                {
                    ROS_INFO_STREAM("waypoint finished service successfully setup for robot: "<<m_status.GetID());
                    break;
                }
                else
                {
                    ROS_ERROR_STREAM("waypoint finished service FAILED to set up for robot: "<<m_status.GetID());
                }
            }
            else
            {
                ROS_ERROR_STREAM("waypoint waitForService timeout occured for robot: "<<m_status.GetID());
            }
        }

        while (true)
        {
            // Setup dump finished service client
            bool success = ros::service::waitForService(Conversion::RobotIDToDumpFinishedTopic(m_status.GetID()), ros::Duration(5));
            if (success)
            {
                m_dumpFinishedPub = m_nh->serviceClient<global_planner::DumpFinished>(Conversion::RobotIDToDumpFinishedTopic(m_status.GetID()), false);
                if (m_dumpFinishedPub.isValid())
                {
                    ROS_INFO_STREAM("dump finished service successfully setup for robot: "<<m_status.GetID());
                    break;
                }
                else
                {
                    ROS_ERROR_STREAM("dump finished service FAILED to set up for robot: "<<m_status.GetID());
                }
            }
            else
            {
                ROS_ERROR_STREAM("dump waitForService timeout occured for robot: "<<m_status.GetID());
            }
        }

        ros::spinOnce();
    }

    // Publishers to send human interface output
    m_soundPub = m_nh->advertise<std_msgs::String>("/interface_sound", 100);
    m_textPub = m_nh->advertise<std_msgs::String>("/interface_text", 100);

    ROS_INFO_STREAM("Finished setting up topics and services");
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

    switch(m_status.GetState())
    {
        case RobotState::WAITING:
        case RobotState::WAITING_TAG_FINISHED:
        case RobotState::NAVIGATING:
        case RobotState::NAVIGATING_TAG_FINISHED:
            // m_tagProcessor->SetShouldPause(false);
            break;
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
            action_client_ptr->cancelAllGoals();
            break;
        case RobotState::NAVIGATING:
            action_client_ptr->cancelAllGoals();
            action_client_ptr->sendGoal(m_moveBaseGoal);
            break;
        case RobotState::NAVIGATING_TAG_SPOTTED:
            ROS_INFO_STREAM("Cancelling goals due to april tag processor");
            action_client_ptr->cancelAllGoals();
            break;
        case RobotState::DUMPING:
            ROS_INFO_STREAM("Starting navigation to dump site.");
            action_client_ptr->sendGoal(m_moveBaseGoal);
            break;
        case RobotState::DUMPING_FINISHED:
            ROS_INFO_STREAM("Reached Dump Site");
            SendDumpFinished(TaskResult::SUCCESS);
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
    //      IF received a dump message: transition(DUMPING)
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
                SendText("should pause - Waiting");
                // SendSound("mario_pause.wav");
                Transition(RobotState::WAITING_TAG_SPOTTED);
            }
        break;

        //In this state, the robot should now be stopping and getting a more accurate view of the tag
        case RobotState::WAITING_TAG_SPOTTED:
            ros::spinOnce();
            execResult = m_tagProcessor->Execute();
            // ROS_DEBUG_STREAM_THROTTLE(0.5, "Result from execute: "<<execResult);
            if (m_tagProcessor->ShouldResume())
            {
                //Transition back to the navigating state, using the same goal as before
                SendText("should resume - WAITING_TAG_SPOTTED");
                ROS_INFO("Resuming robot");
                Transition(RobotState::WAITING_TAG_FINISHED);
                break;
            }
            else
            {
                ROS_INFO_STREAM_THROTTLE(1, "Waiting on the OK to resume from the tag processor");
            }

            if (ros::Time::now() - m_timeEnteringState > ros::Duration(5))
            {
                ROS_ERROR_STREAM("ERROR: could not find any tags while stopped.  we are going to just resume WAITING");
                Transition(RobotState::NAVIGATING);
            }
        break;

        case RobotState::WAITING_TAG_FINISHED:
            if (ros::Time::now() - m_timeEnteringState > ros::Duration(1.0))
            {
                SendSound("mario_pause.wav");
                SendText("resuming - WAITING_TAG_FINISHED");
                Transition(RobotState::WAITING);
                m_tagProcessor->SetShouldPause(false);
            }
            break;

        case RobotState::NAVIGATING:
            if (m_tagProcessor->ShouldPause())
            {
                Transition(RobotState::NAVIGATING_TAG_SPOTTED);
                // SendSound("mario_pause.wav");
            }
            else
            {
                actionlib::SimpleClientGoalState::StateEnum result = action_client_ptr->getState().state_;
                switch (result)
                {
                    case actionlib::SimpleClientGoalState::SUCCEEDED:
                        ROS_INFO_STREAM("Movebase reached target (task id = "<<m_status.GetTaskID()<<")");
                        SendWaypointFinished(TaskResult::SUCCESS);
                        if (m_status.GetTaskID() < WAYPOINT_START_ID)
                        {
                            m_status.IncrementStorageUsed();
                        }
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
            ros::spinOnce();
            execResult = m_tagProcessor->Execute();
            // ROS_DEBUG_STREAM_THROTTLE(0.5, "Result from execute: "<<execResult);
            if (m_tagProcessor->ShouldResume())
            {
                //Transition back to the navigating state, using the same goal as before
                SendText("Resuming robot");
                ROS_INFO("Resuming robot");
                SendText("should resume- NAVIGATING_TAG_SPOTTED");
                Transition(RobotState::NAVIGATING_TAG_FINISHED);
            }
            else
            {
                ROS_INFO_STREAM_THROTTLE(1, "Waiting on the OK to resume from the tag processor");
            }

            if (ros::Time::now() - m_timeEnteringState > ros::Duration(5))
            {
                ROS_ERROR_STREAM("Could not find any tags while stopped.  we are going to just resume NAVIGATING");
                Transition(RobotState::NAVIGATING);
                m_tagProcessor->SetShouldPause(false);
            }
        break;

        case RobotState::NAVIGATING_TAG_FINISHED:
            if (ros::Time::now() - m_timeEnteringState > ros::Duration(2))
            {
                SendSound("mario_pause.wav");
                SendText("resuming - NAVIGATING_TAG_FINISHED");
                ROS_INFO_STREAM("Transitioning back to navigating");
                Transition(RobotState::NAVIGATING);
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


void RobotController::SendSound(std::string filename)
{
    std_msgs::String s;
    s.data = filename;
    m_soundPub.publish(s);
}


void RobotController::SendText(std::string text)
{
    std_msgs::String s;
    s.data = text;
    m_textPub.publish(s);
}
