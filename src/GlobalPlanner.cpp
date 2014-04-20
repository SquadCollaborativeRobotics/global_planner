#include <global_planner/GlobalPlanner.h>
#include <math.h> /* sqrt */
// #include <string>

GlobalPlanner::GlobalPlanner()
{
    m_nh = 0;
}

GlobalPlanner::~GlobalPlanner()
{
}

// Call setup functions
bool GlobalPlanner::Init(ros::NodeHandle* nh)
{
    ROS_INFO_STREAM("Initializing global planner class");

    m_nh = nh;
    SetupCallbacks();

    ROS_INFO_STREAM("Initializing TaskMaster");
    std::string waypointFile("testList1.points");
    if (m_nh->getParam("/global_planner/waypoints_file", waypointFile))
    {
        ROS_INFO_STREAM("Read from file: "<<waypointFile);
    }

    // Planner types :
    // naive : original, for each waypoint in order of file, choose first available robot
    // closest_robot : for each waypoint in order of file, choose closest available robot
    // closest_waypoint : for each available robot in order of id, choose closest available waypoint
    std::string planner("naive");
    m_planner = PLANNER_NAIVE;
    if (m_nh->getParam("/global_planner/planner", planner))
    {
        ROS_INFO_STREAM("Planner set from file to " << planner);
        if (planner.compare("closest_robot") == 0) {
            m_planner = PLANNER_CLOSEST_ROBOT;
        }
        else if (planner.compare("closest_waypoint") == 0) {
            m_planner = PLANNER_CLOSEST_WAYPOINT;
        }
	}
    m_tm.Init(nh, m_robots, waypointFile);

    ros::spinOnce();

    return true;
}

void GlobalPlanner::Display()
{
    m_lastDisplay = ros::Time::now();
    ROS_INFO_STREAM("Robot Status:");
    for (std::map<int, Robot_Ptr>::iterator it = m_robots.begin(); it != m_robots.end(); ++it)
    {
        ROS_INFO_STREAM(it->second->ToString());
    }

    geometry_msgs::PoseArray availPoseArray;
    geometry_msgs::PoseArray finPoseArray;
    availPoseArray.header.stamp = ros::Time::now();
    availPoseArray.header.frame_id = "/map";
    finPoseArray.header = availPoseArray.header;


    std::map<int, Waypoint_Ptr> wps = m_tm.GetWaypoints();
    if (wps.size() > 0)
    {
        ROS_INFO_STREAM("Waypoint Status:");
    }
    for (std::map<int, Waypoint_Ptr>::iterator it = wps.begin(); it != wps.end(); ++it)
    {
        ROS_INFO_STREAM(it->second->ToString());
        if (it->second->GetStatus() == TaskResult::SUCCESS)
            finPoseArray.poses.push_back(it->second->GetPose());
        else
            availPoseArray.poses.push_back(it->second->GetPose());
    }
    m_waypointPoseArrayAvailPub.publish(availPoseArray);
    m_waypointPoseArrayFinPub.publish(finPoseArray);

    finPoseArray.poses.clear();
    availPoseArray.poses.clear();

    std::map<int, Goal_Ptr> goals = m_tm.GetGoals();
    if (goals.size() > 0)
    {
        ROS_INFO_STREAM("Goal Status");
    }
    for (std::map<int, Goal_Ptr>::iterator it = goals.begin(); it != goals.end(); ++it)
    {
        ROS_INFO_STREAM(it->second->ToString());
        if (it->second->GetStatus() == TaskResult::SUCCESS)
            finPoseArray.poses.push_back(it->second->GetPose());
        else
            availPoseArray.poses.push_back(it->second->GetPose());
    }

    m_goalPoseArrayAvailPub.publish(availPoseArray);
    m_goalPoseArrayFinPub.publish(finPoseArray);

    finPoseArray.poses.clear();
    availPoseArray.poses.clear();


    std::map<int, Dump_Ptr> dumps = m_tm.GetDumps();
    if (dumps.size() > 0)
    {
        ROS_INFO_STREAM("Dump Status");
    }
    for (std::map<int, Dump_Ptr>::iterator it = dumps.begin(); it != dumps.end(); ++it)
    {
        ROS_INFO_STREAM(it->second->ToString());
        if (it->second->GetStatus() == TaskResult::SUCCESS)
        {
            finPoseArray.poses.push_back(it->second->GetPose1());
            finPoseArray.poses.push_back(it->second->GetPose2());
        }
        else
        {
            availPoseArray.poses.push_back(it->second->GetPose1());
            availPoseArray.poses.push_back(it->second->GetPose2());
        }
    }
    m_dumpPoseArrayAvailPub.publish(availPoseArray);
    m_dumpPoseArrayFinPub.publish(finPoseArray);
}

// Executive function
void GlobalPlanner::Execute()
{
    ros::spinOnce();

    if ((ros::Time::now() - m_lastDisplay) > ros::Duration(3))
    {
        Display();
    }

    // Get Robot Status...
    QueryRobots();

    // FOR each pair of robots that're just chillin in a dump stage and are stopped near the handoff location
    //      Pick best bot to meet with?
    //      Check if robot1 & robot2 are within handoff threshold
    //      Add new dump to m_dumpMap
    //      Send handoff message to each robot giving them their new capacities
    //
    // IF robot in wait state AND is full AND there are goals
    //      Find the best dump location for the robots to meet
    //      Update Dump List
    //      Send Dump Message
    //
    // IF there are goals available
    //      pick best robot to do job (return an ID)
    //      OPTIONAL: IF need to cancel robot's current goal -> Send Cancel Message First AND update the task it was assigned to
    //      Send goal message
    // ELSE IF robots available AND waypoints available
    //      pick best robot to get to waypoints (return an ID)
    //      OPTIONAL: IF need to cancel robot's current goal -> Send Cancel Message First AND update the task it was assigned to
    //      Send Waypoint Message

    /*
    // Check if a robot is full & find the best binbot for it
    std::vector<Robot_Ptr> availableRobots = GetAvailableRobots();
    for (std::vector<Robot_Ptr>::iterator it = availableRobots.begin(); it != availableRobots.end(); ++it)
    {
        Robot_Ptr robot = *it;
        // If robot has no space (TODO: Check if collector bot or bin bot for where to dump)
        if (robot->GetStorageAvailable() <= 0 &&
            robot->GetType() == RobotState::COLLECTOR_BOT)
        {
            ROS_INFO_STREAM_THROTTLE(5, "Robot " << robot->GetName() <<
                            "(" << robot->GetID() << ")" <<
                            " full, trying to find bin bot to dump to...");
            int collectorBot = robot->GetID();
            // Get closest bin bot to collector bot if it exists
            int bestBinBot = GetBestBinBot( collectorBot );

            if (bestBinBot != NO_ROBOT_FOUND) {
                ROS_INFO_STREAM("Bin Bot " << robot->GetName() <<
                                "(" << robot->GetID() << ") found.");
                // Create new dump site
                Dump_Ptr dp(new DumpWrapper());
                m_tm.AddDump(dp);
                // Assign collector robot to dump, Assign bin bot to dump
                if (!AssignRobotsDump(collectorBot, bestBinBot, dp->GetID()))
                {
                    ROS_ERROR_STREAM("Error Assigning Robots " << m_robots[collectorBot]->GetName() <<
                                     "(" << collectorBot << ") & " << m_robots[bestBinBot]->GetName() <<
                                     "(" << bestBinBot << ")" << " to Dump(" << dp->GetID() << ")");
                }
            }
        }
    }

    // Check each dump, see if each robot in the dump finished state
    std::map<int, Dump_Ptr > dumps = m_tm.GetDumps();
    for (std::map<int, Dump_Ptr>::iterator it = dumps.begin(); it != dumps.end(); ++it)
    {
        Dump_Ptr dump = it->second;
        if (dump->GetReadyToTransfer()) {
            ROS_INFO_STREAM("Dump(" << dump->GetID() << ") transfering trash.");

            // Move trash over
            int collectorBot = dump->GetRobot1();
            int binBot = dump->GetRobot2();
            int trash_to_transfer = m_robots[collectorBot]->GetStorageUsed();

            // Send trash updates to robot controllers
            global_planner::SetTrashSrv s;
            s.request.storage = 0;
            if (m_setTrashServices[collectorBot].call(s) && s.response.result == 0) { ROS_INFO_STREAM("Set Trash for " << m_robots[collectorBot]->GetName()); }
            else { ROS_ERROR_STREAM("Did not receive trash response from robot: " << collectorBot); }

            s.request.storage = m_robots[binBot]->GetStorageUsed() + trash_to_transfer;
            if (m_setTrashServices[binBot].call(s) && s.response.result == 0) { ROS_INFO_STREAM("Set Trash for " << m_robots[binBot]->GetName()); }
            else { ROS_ERROR_STREAM("Did not receive trash response from robot: " << binBot); }

            // In global planner, update storage values
            // Add trash to bin bot
            m_robots[binBot]->SetStorageUsed( m_robots[binBot]->GetStorageUsed() + trash_to_transfer );
            // Remove trash from collector bot
            m_robots[collectorBot]->SetStorageUsed(0);

            // Set states to waiting in global planner but not robot controllers
            // the robot controllers are in DUMP_FINISHED state, but they're ready to transition to waypoints etc.
            m_robots[collectorBot]->SetState(RobotState::WAITING);
            m_robots[binBot]->SetState(RobotState::WAITING);

            // Transition dump state to success
            dump->SetStatus(TaskResult::SUCCESS);
        }
    }
    */

    ProcessGoals();

    /*
    // If no available waypoints, do nothing
    if (m_tm.GetAvailableWaypoints().size() == 0) {
        ROS_WARN_STREAM_THROTTLE(10, "No Available Waypoints! ("
                                     << m_tm.GetAvailableWaypoints().size() << ") "
                                     << (isFinished() ? "FIN" : "GP Waiting") );

        // Print out status of all waypoints
        std::map<int, Waypoint_Ptr> allWaypoints = m_tm.GetWaypoints();
        std::stringstream ss;
        for (std::map<int, Waypoint_Ptr>::iterator it = allWaypoints.begin(); it != allWaypoints.end(); ++it)
        {
            ss << it->second->GetID() << "-" << it->second->GetStatusMessage() << " ";
        }
        ROS_INFO_STREAM_THROTTLE(10, ss.str() );
        return;
    }
    */

    switch (m_planner)
    {
        case PLANNER_CLOSEST_ROBOT:
        PlanNNRobot();
        break;

        case PLANNER_CLOSEST_WAYPOINT:
        PlanNNWaypoint();
        break;

        case PLANNER_NAIVE:
        default:
        PlanNaive();
        break;
    }
}


/***********************************************************************
 *  Method: GlobalPlanner::ProcessGoals
 *  Params:
 * Returns:
 * Effects: Iterate over available goals and choose the best robot to
 *          get the goal
 ***********************************************************************/
void GlobalPlanner::ProcessGoals()
{
    // If there are currently goals still not finished
    std::vector<Goal_Ptr> availableGoals = m_tm.GetAvailableGoals();
    for (std::vector<Goal_Ptr>::iterator goal_it = availableGoals.begin(); goal_it != availableGoals.end(); ++goal_it)
    {
        if (ros::Time::now() - (*goal_it)->GetTime() < ros::Duration(2.5))
        {
            ROS_INFO_THROTTLE(1.0, "Waiting for the sending robot to be ready to accept this goal");
            continue;
        }
        geometry_msgs::Pose goalPose = (*goal_it)->GetPose();
        int closestRobot = NO_ROBOT_FOUND;
        double best_dist = MAX_DIST;
        for (std::map<int, Robot_Ptr>::iterator robot_it = m_robots.begin(); robot_it != m_robots.end(); ++robot_it)
        {
            //See if the robot is available
            if (robot_it->second->GetState() == RobotState::WAITING ||
                robot_it->second->GetState() == RobotState::NAVIGATING)
            {
                geometry_msgs::Pose robotPose = robot_it->second->GetPose();
                double dist = Get2DPoseDistance(robotPose, goalPose);
                if (dist < best_dist)
                {
                    best_dist = dist;
                    closestRobot = robot_it->first;
                }
            }
        }
        if (closestRobot != NO_ROBOT_FOUND)
        {
            // Assign Robot to goal
            if (!AssignRobotGoal(closestRobot , (*goal_it)->GetID()))
            {
                ROS_ERROR_STREAM("Error Assigning Robot " << m_robots[closestRobot]->GetName() << "(" << m_robots[closestRobot]->GetID() << ")"
                                             << " - Goal(" << (*goal_it)->GetID() << ")");
            }
            else
            {
                ROS_INFO_STREAM("Success sending goal "<<(*goal_it)->ToString());
            }
        }
        else
        {
            ROS_INFO_STREAM_THROTTLE(1.0, "No robots available to get goal["<<(*goal_it)->GetID()<<"]");
        }
    }
}


/***********************************************************************
 *  Method: GlobalPlanner::PlanNNWaypoint
 *  Params:
 * Returns:
 * Effects: Iterate over available robots and choose the nearest available waypoint
 ***********************************************************************/
void GlobalPlanner::PlanNNWaypoint()
{
    std::vector<Robot_Ptr> availableRobots = GetAvailableRobots();
    std::map<int, Waypoint_Ptr> allWaypoints = m_tm.GetWaypoints();
    std::vector<Waypoint_Ptr> availableWaypoints = m_tm.GetAvailableWaypoints();
    // Break if all waypoints reached.
    if (availableWaypoints.size() == 0) { return; }


    for (std::vector<Robot_Ptr>::iterator i = availableRobots.begin(); i != availableRobots.end(); ++i)
    {
        // Get updated set of available waypoints each time
        availableWaypoints = m_tm.GetAvailableWaypoints();
        ROS_INFO_STREAM("Waypoints to go: (" << availableWaypoints.size() << ")");

        // Break if all waypoints reached.
        if (availableWaypoints.size() == 0) {
            break;
        }
        Robot_Ptr robot = *i;

        int waypoint_id = GetWaypointClosestToRobot(robot->GetID());
        if (waypoint_id == NO_WAYPOINT_FOUND) {
            ROS_WARN("No Waypoint Found!");
            break;
        }

        Waypoint_Ptr bestwp = allWaypoints[waypoint_id];

        // Assign Robot to Waypoint
        if (!AssignRobotWaypoint(robot->GetID() , waypoint_id))
        {
            ROS_ERROR_STREAM_THROTTLE(1, "Error Assigning Robot " << robot->GetName() << "(" << robot->GetID() << ")"
                                         << " - Waypoint(" << waypoint_id << ")");
        }

        // Print out waypoints and their statuses
        for (std::map<int, Waypoint_Ptr>::iterator it = allWaypoints.begin(); it != allWaypoints.end(); ++it)
        {
            ROS_INFO_STREAM(it->second->ToShortString());
        }
    }
}


/***********************************************************************
 *  Method: GlobalPlanner::AssignRobotWaypoint
 *  Params: int robot_id, int waypoint_id
 * Returns: true if successful
 * Effects: Sets robot to navigating state and sends waypoint. Sets waypoint robot and status to in progress
 ***********************************************************************/
bool GlobalPlanner::AssignRobotWaypoint(int robot_id, int waypoint_id)
{
    Waypoint_Ptr wp = m_tm.GetWaypoints()[waypoint_id];
    ROS_INFO_STREAM("Assigning Robot " << m_robots[robot_id]->GetName() << "(" << robot_id << ") : Waypoint ("<< waypoint_id <<")" );

    // Assign waypoint to robot
    wp->SetRobot(robot_id);

    // Assign robot to best waypoint
    if (m_tm.SendWaypoint(waypoint_id))
    {
        //Update lists of waypoints/robots
        m_robots[robot_id]->SetState(RobotState::NAVIGATING);
        wp->SetStatus(TaskResult::INPROGRESS);
        // Track assignment in statistics
        // map<robot_id, map<waypoint_id, double_seconds_since_start> >
        robot_waypoint_times[robot_id].insert(std::pair<int,double> (waypoint_id, TimeSinceStart() ) );
        // robot_waypoint_times.insert(std::pair<int, std::map<int,double> >(robot_id,));
        ROS_INFO_STREAM("Successfully sent waypoint to robot "<<robot_id);
    }
    else
    {
        wp->SetRobot(NO_ROBOT_FOUND);
        wp->SetStatus(TaskResult::COMM_FAILURE);
        ROS_ERROR_STREAM("Could not assign waypoint["<<waypoint_id<<"] to robot ["<<robot_id<<"]");
    }

    return true;
}

/***********************************************************************
 *  Method: GlobalPlanner::AssignRobotsDump
 *  Params: int robot_id, int dump_id
 * Returns: true if successful
 * Effects: Assign two robots to a dump site to meet
 ***********************************************************************/
bool GlobalPlanner::AssignRobotsDump(int collector_robot_id, int bin_robot_id, int dump_id)
{
    Dump_Ptr dump_ptr = m_tm.GetDumps()[dump_id];
    ROS_INFO_STREAM("Assigning Robots " <<
        m_robots[collector_robot_id]->GetName() << "(" << collector_robot_id << ") & " <<
        m_robots[bin_robot_id]->GetName() << "(" << bin_robot_id << ") to dump(" << dump_id << ")" );

    // Assign dump to robot
    dump_ptr->SetRobot1(collector_robot_id);
    dump_ptr->SetRobot2(bin_robot_id);
    dump_ptr->SetPose1(Conversion::SetPose(2,0,1,0)); // TODO : choose from list of locations
    dump_ptr->SetPose2(Conversion::SetPose(2,3,1,0));
    // dump_ptr->SetPose1(Conversion::SetPose(2,-3,1,0)); // TODO : choose from list of locations
    // dump_ptr->SetPose2(Conversion::SetPose(2,-2,0,1));
    dump_ptr->SetTime(ros::Time::now());

    // Assign robot to best dump
    if (m_tm.SendDump(dump_id))
    {
        // Update lists of dumps/robots
        m_robots[collector_robot_id]->SetState(RobotState::DUMPING);
        m_robots[bin_robot_id]->SetState(RobotState::DUMPING);
        dump_ptr->SetStatus(TaskResult::INPROGRESS);

        ROS_INFO_STREAM("Successfully sent dump (" << dump_id<<") to robots [" << collector_robot_id << "] & [" << bin_robot_id << "]");
    }
    else
    {
        dump_ptr->SetRobot1(NO_ROBOT_FOUND);
        dump_ptr->SetRobot2(NO_ROBOT_FOUND);
        dump_ptr->SetStatus(TaskResult::COMM_FAILURE);
        ROS_ERROR_STREAM("Could not assign dump[" << dump_id<<"] to robots [" << collector_robot_id << "] & [" << bin_robot_id << "]");
        return false;
    }



    return true;
}

/***********************************************************************
 *  Method: GlobalPlanner::AssignRobotGoal
 *  Params: int robot_id, int goal_id
 * Returns: true if successful
 * Effects: Sets robot to collecting state and sends goal. Sets goal robot and status to in progress
 ***********************************************************************/
bool GlobalPlanner::AssignRobotGoal(int robot_id, int goal_id)
{
    Goal_Ptr goal_ptr = m_tm.GetGoals()[goal_id];
    ROS_INFO_STREAM("Assigning Robot " << m_robots[robot_id]->GetName() << "(" << robot_id << ") : Goal ("<< goal_id <<")" );

    // Assign goal to robot
    goal_ptr->SetRobot(robot_id);

    // Assign robot to best goal
    if (m_tm.SendGoal(goal_id))
    {
        //Update lists of goals/robots
        m_robots[robot_id]->SetState(RobotState::COLLECTING);
        goal_ptr->SetStatus(TaskResult::INPROGRESS);
        // Track assignment in statistics
        // map<robot_id, map<goal_id, double_seconds_since_start> >
        robot_goal_times[robot_id].insert(std::pair<int,double> (goal_id, TimeSinceStart() ) );
        // robot_goal_times.insert(std::pair<int, std::map<int,double> >(robot_id,));
        ROS_INFO_STREAM("Successfully sent goal ("<<goal_id<<") to robot "<<robot_id);
    }
    else
    {
        goal_ptr->SetRobot(NO_ROBOT_FOUND);
        goal_ptr->SetStatus(TaskResult::COMM_FAILURE);
        ROS_ERROR_STREAM("Could not assign goal["<<goal_id<<"] to robot ["<<robot_id<<"]");
    }

    return true;
}


/***********************************************************************
 *  Method: GlobalPlanner::PlanNNRobot
 *  Params:
 * Returns:
 * Effects: Iterate over waypoints and choose the nearest available robot
 ***********************************************************************/
void GlobalPlanner::PlanNNRobot()
{
    std::vector<Waypoint_Ptr> availableWaypoints = m_tm.GetAvailableWaypoints();

    for (std::vector<Waypoint_Ptr>::iterator it = availableWaypoints.begin(); it != availableWaypoints.end(); ++it)
    {
        Waypoint_Ptr wp = *it;
        int bestRobot = GetBestSearchBot(wp->GetID()); // Nearest Robot to waypoint SOLUTION
        // int bestRobot = GetFirstAvailableBot(); // NAIVE SOLUTION
        if (bestRobot != NO_ROBOT_FOUND)
        {
            // ROS_INFO_STREAM("Found a robot to explore waypoint ("<<wp->GetID()<<") : "<<bestRobot);
            // Assign Robot to Waypoint
            if ( !AssignRobotWaypoint(bestRobot , wp->GetID()) )
            {
                ROS_ERROR_STREAM_THROTTLE(5, "Error Assigning Robot " << m_robots[bestRobot]->GetName() << "(" << m_robots[bestRobot]->GetID() << ")"
                                             << " - Waypoint(" << wp->GetID() << ")");
            }
        }
        else
        {
            // ROS_INFO_THROTTLE(5, "FAILED TO FIND A ROBOT");
        }
    }
}

/***********************************************************************
 *  Method: GlobalPlanner::PlanNaive
 *  Params:
 * Returns:
 * Effects: Naively iterates over waypoints in order and  assigns first available robot to each waypoint
 ***********************************************************************/
void GlobalPlanner::PlanNaive()
{
    std::vector<Waypoint_Ptr> availableWaypoints = m_tm.GetAvailableWaypoints();

    for (std::vector<Waypoint_Ptr>::iterator it = availableWaypoints.begin(); it != availableWaypoints.end(); ++it)
    {
        Waypoint_Ptr wp = *it;
        // int bestRobot = GetBestSearchBot(wp->GetID()); // Nearest Robot to waypoint SOLUTION
        int bestRobot = GetFirstAvailableBot(); // NAIVE SOLUTION
        if (bestRobot != NO_ROBOT_FOUND)
        {
            // ROS_INFO_STREAM("Found a robot to explore waypoint ("<<wp->GetID()<<") : "<<bestRobot);
            // Assign Robot to Waypoint
            if ( !AssignRobotWaypoint(bestRobot , wp->GetID()) )
            {
                ROS_ERROR_STREAM_THROTTLE(5, "Error Assigning Robot " << m_robots[bestRobot]->GetName() << "(" << m_robots[bestRobot]->GetID() << ")"
                                             << " - Waypoint(" << wp->GetID() << ")");
            }
        }
        else
        {
            // ROS_INFO_THROTTLE(5, "FAILED TO FIND A ROBOT");
        }
    }
}

/***********************************************************************
 *  Method: GlobalPlanner::GetAvailableRobots
 *  Params:
 * Returns: std::vector<Robot_Ptr> of all available robots
 * Effects: Returns all robots that are in the available state (even with no storage space)
 ***********************************************************************/
std::vector<Robot_Ptr> GlobalPlanner::GetAvailableRobots()
{
    return GetAvailableRobots(0);
}
// Get Available robots with at least the number of available storage
std::vector<Robot_Ptr> GlobalPlanner::GetAvailableRobots(int available_storage)
{
    std::vector<Robot_Ptr> v;
    for (std::map<int, Robot_Ptr>::iterator it = m_robots.begin(); it != m_robots.end(); ++it)
    {
        // If it's in the "waiting" state and has the required storage, it's avilable for task setting
        if (it->second->GetState() == RobotState::WAITING &&
            it->second->GetStorageAvailable() >= available_storage)
        {
            v.push_back(it->second);
        }
    }
    return v;
}

/***********************************************************************
 *  Method: GlobalPlanner::GetBestBinBot
 *  Params: (int idOfRobotThatNeedsIt)
 * Returns: int id of robot
 * Effects: Wrapper passing first available bin bot
 ***********************************************************************/
int GlobalPlanner::GetBestBinBot(int idOfRobotThatNeedsIt)
{
    return GetRobotClosestToRobot(idOfRobotThatNeedsIt, RobotState::BIN_BOT);
}

/***********************************************************************
 *  Method: GlobalPlanner::GetBestCollectorbot
 *  Params: (int goalID)
 * Returns: int id of robot
 * Effects: Returns the id of the best collector bot for given goal id
 ***********************************************************************/
int GlobalPlanner::GetBestCollectorbot(int goalID)
{
    return GetRobotClosestToGoal(goalID, RobotState::COLLECTOR_BOT);
}
/***********************************************************************
 *  Method: GlobalPlanner::GetBestSearchBot
 *  Params: int waypointID
 * Returns: int id of robot closest to waypoint (of any type)
 * Effects: Wrapper on GetRobotClosestToWaypoint with any type
 ***********************************************************************/
int GlobalPlanner::GetBestSearchBot(int waypointID)
{
    // return GetFirstAvailableBot();
    return GetRobotClosestToWaypoint(waypointID, RobotState::ANY);
}

/***********************************************************************
 *  Method: GlobalPlanner::GetFirstAvailableBot
 *  Params: int waypointID
 * Returns: int id of robot
 * Effects: Returns the first available robot of any type that is available (waiting and has storage space)
 ***********************************************************************/
int GlobalPlanner::GetFirstAvailableBot()
{
    return GetFirstAvailableBot(RobotState::ANY);
}

/***********************************************************************
 *  Method: GlobalPlanner::GetFirstAvailableBot
 *  Params: GlobalPlanner::ROBOT_TYPE type
 * Returns: int id of robot
 * Effects: Returns the first available robot of the right type that is available (waiting and has storage space)
 ***********************************************************************/
int GlobalPlanner::GetFirstAvailableBot(RobotState::Type type)
{
    // For all robots
    for (std::map<int, Robot_Ptr>::iterator it = m_robots.begin(); it != m_robots.end(); ++it)
    {
        // If robot is waiting, is right kind, and has storage space
        if (it->second->GetState() == RobotState::WAITING &&  // Robot is available
            (type == RobotState::ANY || it->second->GetType() == type) && // And robot is right kind (any or collector or bin)
            it->second->GetStorageAvailable() > 0) // And robot has storage space
        {
            // Choose the first match
            return it->first;
        }
    }
    return NO_ROBOT_FOUND;
}

/***********************************************************************
 *  Method: GlobalPlanner::GetRobotClosestToRobot
 *  Params: int robotID, GlobalPlanner::ROBOT_TYPE type
 * Returns: int id of robot
 * Effects: Returns the physically closest robot of the right type that is available (waiting and has storage space)
 ***********************************************************************/
int GlobalPlanner::GetRobotClosestToRobot(int robotID, RobotState::Type type)
{
    return GetRobotClosestToPose(m_robots[robotID]->GetPose(), type);
}

/***********************************************************************
 *  Method: GlobalPlanner::GetRobotClosestToWaypoint
 *  Params: int waypointID, GlobalPlanner::ROBOT_TYPE type
 * Returns: int id of robot
 * Effects: Returns the physically closest robot of the right type that is available (waiting and has storage space)
 ***********************************************************************/
int GlobalPlanner::GetRobotClosestToWaypoint(int waypointID, RobotState::Type type)
{
    std::map<int, Waypoint_Ptr> waypoints = m_tm.GetWaypoints();
    return GetRobotClosestToPose(waypoints[waypointID]->GetPose(), type);
}

/***********************************************************************
 *  Method: GlobalPlanner::GetRobotClosestToGoal
 *  Params: int goalID, GlobalPlanner::ROBOT_TYPE type
 * Returns: int id of robot
 * Effects: Returns the physically closest robot of the right type that is available (waiting and has storage space)
 ***********************************************************************/
int GlobalPlanner::GetRobotClosestToGoal(int goalID, RobotState::Type type)
{
    std::map<int, Goal_Ptr> goals = m_tm.GetGoals();
    return GetRobotClosestToPose(goals[goalID]->GetPose(), type);
}

/***********************************************************************
 *  Method: GlobalPlanner::GetRobotClosestToPose
 *  Params: geometry_msgs::Pose pose, GlobalPlanner::ROBOT_TYPE type
 * Returns: int id of robot
 * Effects: Returns the physically closest robot of the right type that is available (waiting and has storage space)
 ***********************************************************************/
int GlobalPlanner::GetRobotClosestToPose(geometry_msgs::Pose pose, RobotState::Type type)
{
    int best_robot_id = NO_ROBOT_FOUND;
    double best_distance = MAX_DIST;

    // For all robots
    for (std::map<int, Robot_Ptr>::iterator it = m_robots.begin(); it != m_robots.end(); ++it)
    {
        int robot_id = it->first;
        Robot_Ptr robot = it->second;

        // If robot is waiting, is right kind, and has storage space
        if (robot->GetState() == RobotState::WAITING &&  // Robot is available
            (type == RobotState::ANY || robot->GetType() == type) && // And robot is right kind (any or collector or bin)
            robot->GetStorageAvailable() > 0) // And robot has storage space
        {
            // Get robot distance to waypoint
            double dist = Get2DPoseDistance(robot->GetPose(), pose);

            // If closest robot so far, update
            if (dist < best_distance)
            {
                best_distance = dist;
                best_robot_id = robot_id;
            }
        }
    }
    return best_robot_id;
}


// Returns waypoint id closest to robot id
int GlobalPlanner::GetWaypointClosestToRobot(int robot_id) {
    // List of available waypoints
    std::vector<Waypoint_Ptr> availableWaypoints = m_tm.GetAvailableWaypoints();

    // Break if all waypoints reached.
    if (availableWaypoints.size() == 0) { return NO_WAYPOINT_FOUND; }

    double best_distance = MAX_DIST;

    // Robot pose
    geometry_msgs::Pose robot_pose = m_robots[robot_id]->GetPose();

    // idx of best waypoint so far
    int best_waypoint_id = NO_WAYPOINT_FOUND;

    for (std::vector<Waypoint_Ptr>::iterator it = availableWaypoints.begin(); it != availableWaypoints.end(); ++it)
    {
        Waypoint_Ptr wp = *it;
        geometry_msgs::Pose waypoint_pose = wp->GetPose();
        // Get robot distance to waypoint
        double dist = Get2DPoseDistance(robot_pose, waypoint_pose);

        // If closest robot so far, update
        if (dist < best_distance)
        {
            best_distance = dist;
            best_waypoint_id = wp->GetID();
        }
    }
    return best_waypoint_id;
}

// System Start
void GlobalPlanner::Start()
{
    m_start_time = ros::Time::now();
    SendSound("mario_lets_go.wav");
    std::stringstream ss;
    ss << "Starting global planner";
    SendText(ss.str());
}


// System finished
void GlobalPlanner::Finished()
{
    SendSound("mario_world_clear.wav");
    std::stringstream ss;
    ss << "Global Planner finished";
    SendText(ss.str());

    // TODO: Wrap up statistics here.
    ROS_INFO("Global Planner finished in : %g seconds", TimeSinceStart() );

    // Print out waypoint assignment table
    ROS_INFO_STREAM("Robots - Waypoint Assignment Table:");
    for (std::map<int, std::map<int, double> >::iterator robot_it = robot_waypoint_times.begin();
         robot_it != robot_waypoint_times.end();
         ++robot_it)
    {
        std::stringstream ss;
        for (std::map<int, double>::iterator waypoint_it = robot_it->second.begin(); waypoint_it != robot_it->second.end(); ++waypoint_it)
        {
            ss << "[WP " << waypoint_it->first << ": " << waypoint_it->second << "s] ";
        }
        ROS_INFO_STREAM(m_robots[robot_it->first]->GetName() << "(" << robot_it->first << ") : " << ss.str());
    }

    // Print out goal assignment table
    ROS_INFO_STREAM("Robots - Goal Assignment Table:");
    for (std::map<int, std::map<int, double> >::iterator robot_it = robot_goal_times.begin();
         robot_it != robot_goal_times.end();
         ++robot_it)
    {
        std::stringstream ss;
        for (std::map<int, double>::iterator goal_it = robot_it->second.begin(); goal_it != robot_it->second.end(); ++goal_it)
        {
            ss << "[GOAL " << goal_it->first << ": " << goal_it->second << "s] ";
        }
        ROS_INFO_STREAM(m_robots[robot_it->first]->GetName() << "(" << robot_it->first << ") : " << ss.str());
    }
}

// True if all waypoints chomped (success/failure/abort of any kind), false if any are still available or in progress
bool GlobalPlanner::isFinished() {
    return m_tm.isFinished();
}


// setup callbacks, regiser services, load waypoints...
bool GlobalPlanner::SetupCallbacks()
{
    m_robotSub = m_nh->subscribe("/robot_status", 10, &GlobalPlanner::cb_robotStatus, this);
    m_eStopPub = m_nh->advertise<std_msgs::Empty>("/e_stop_pub", 10);

    m_waypointPoseArrayAvailPub = m_nh->advertise<geometry_msgs::PoseArray>("/waypoints_available", 5);
    m_waypointPoseArrayFinPub = m_nh->advertise<geometry_msgs::PoseArray>("/waypoints_finished", 5);
    m_goalPoseArrayAvailPub = m_nh->advertise<geometry_msgs::PoseArray>("/goals_available", 5);
    m_goalPoseArrayFinPub = m_nh->advertise<geometry_msgs::PoseArray>("/goals_finished", 5);
    m_dumpPoseArrayAvailPub = m_nh->advertise<geometry_msgs::PoseArray>("/dumps_available", 5);
    m_dumpPoseArrayFinPub = m_nh->advertise<geometry_msgs::PoseArray>("/dumps_finished", 5);

    // Publishers to send human interface output
    m_soundPub = m_nh->advertise<std_msgs::String>("/interface_sound", 100);
    m_textPub = m_nh->advertise<std_msgs::String>("/interface_text", 100);

    ros::spinOnce();
}


// Get robot information TODO: better definition
void GlobalPlanner::cb_robotStatus(const global_planner::RobotStatus::ConstPtr& msg)
{
    int id = msg->id;
    // ROS_INFO_STREAM_THROTTLE(1, "Received robot status: "<<id);
    global_planner::RobotStatus status = *msg;

    std::map<int, Robot_Ptr>::iterator it = m_robots.find(id);
    //If it is already in the map...
    if(it != m_robots.end())
    {
        m_robots[id]->SetPose(status.pose);
    }
    else
    {
        Robot_Ptr ptr( new RobotStatusWrapper() );
        ptr->SetData(status);
        ptr->SetState(RobotState::UNINITIALIZED);
        m_robots[id] = ptr;

        std::string serviceTopic = Conversion::RobotIDToServiceName(id);
        //create a persistant service with this
        m_statusServices[id] = m_nh->serviceClient<global_planner::RobotStatusSrv>(serviceTopic, true);

        std::string serviceTopic2 = Conversion::RobotIDToSetTrash(id);
        m_setTrashServices[id] = m_nh->serviceClient<global_planner::SetTrashSrv>(serviceTopic2, true);

        m_tm.UpdateRobotMap(m_robots);

        ROS_INFO_STREAM("Added new robot: " << ptr->ToString());
    }
}


/***********************************************************************
 *  Method: GlobalPlanner::SendSound
 *  Params: std::string filename
 * Returns: void
 * Effects: play the sound specified in the filename
 *  (relaative to the 'resources/sounds' folder)
 ***********************************************************************/
void GlobalPlanner::SendSound(std::string filename)
{
    std_msgs::String s;
    s.data = filename;
    m_soundPub.publish(s);
}


/***********************************************************************
 *  Method: GlobalPlanner::SendSound
 *  Params: std::string filename
 * Returns: void
 * Effects: send text to the human interface node
 ***********************************************************************/
void GlobalPlanner::SendText(std::string text)
{
    std_msgs::String s;
    s.data = text;
    m_textPub.publish(s);
}


//// HELPER FUNCTIONS

/***********************************************************************
 *  Method: GlobalPlanner::Get2DPoseDistance
 *  Params: geometry_msgs::Pose a, geometry_msgs::Pose b
 * Returns: double x/y ground distance between two poses
 * Effects: Returns the physically closest robot of the right type that is available (waiting and has storage space)
 ***********************************************************************/
double GlobalPlanner::Get2DPoseDistance(geometry_msgs::Pose a, geometry_msgs::Pose b) {
    double dx = (b.position.x - a.position.x);
    double dy = (b.position.y - a.position.y);
    return sqrt( dx*dx + dy*dy );
}

double GlobalPlanner::TimeSinceStart()
{
    return (ros::Time::now() - m_start_time).toSec();
}


/***********************************************************************
 *  Method: GlobalPlanner::QueryRobots
 *  Params:
 * Returns: void
 * Effects:
 ***********************************************************************/
void GlobalPlanner::QueryRobots()
{
    for (std::map<int, ros::ServiceClient>::iterator it = m_statusServices.begin(); it != m_statusServices.end(); ++it)
    {
        global_planner::RobotStatusSrv s;
        s.request.id = it->first;
        if (it->second)
        {
            if (it->second.call(s))
            {
                m_robots[s.request.id]->SetData(s.response.status);
                ROS_INFO_STREAM_THROTTLE(10.0, "Received response from robot: "<<s.response.status.id<<" : "<<m_robots[s.request.id]->ToString());
            }
            else
            {
                ROS_ERROR_STREAM("Did not receive response from robot: "<<it->first);
            }
        }
        else
        {
            ROS_ERROR_STREAM("Not connected to robot: "<<it->first<<"... retrying");
            std::string serviceTopic = Conversion::RobotIDToServiceName(it->first);
            m_statusServices[it->first] = m_nh->serviceClient<global_planner::RobotStatusSrv>(serviceTopic, true);
        }
    }
}


