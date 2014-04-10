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
    }

    if (planner.compare("closest_robot") == 0) {
        m_planner = PLANNER_CLOSEST_ROBOT;
    }
    else if (planner.compare("closest_waypoint") == 0) {
        m_planner = PLANNER_CLOSEST_WAYPOINT;
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

    ROS_INFO_STREAM("Waypoint Status:");
    std::map<int, Waypoint_Ptr> wps = m_tm.GetWaypoints();
    for (std::map<int, Waypoint_Ptr>::iterator it = wps.begin(); it != wps.end(); ++it)
    {
        ROS_INFO_STREAM(it->second->ToString());
    }

    ROS_INFO_STREAM("Goal Status:");
    std::map<int, Goal_Ptr> goals = m_tm.GetGoals();
    for (std::map<int, Goal_Ptr>::iterator it = goals.begin(); it != goals.end(); ++it)
    {
        ROS_INFO_STREAM(it->second->ToString());
    }

    ROS_INFO_STREAM("Dump Status");
    std::map<int, Dump_Ptr> dumps = m_tm.GetDumps();
    for (std::map<int, Dump_Ptr>::iterator it = dumps.begin(); it != dumps.end(); ++it)
    {
        ROS_INFO_STREAM(it->second->ToString());
    }
}

// Executive function
void GlobalPlanner::Execute()
{
    // ROS_INFO_THROTTLE(5,"Executing...");

    ros::spinOnce();

    if ((ros::Time::now() - m_lastDisplay) > ros::Duration(3))
    {
        //Display();
    }

    // Get Robot Status...
    ROS_INFO("Getting new info from robots");
    QueryRobots();
    ROS_INFO("Finished updating the robots");

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

    // std::vector<Robot_Ptr> availableRobots = GetAvailableRobots();

    //TODO: Handoff

/*
    // Check if a robot is full & find the best binbot for it
    for (std::vector<Robot_Ptr>::iterator it = availableRobots.begin(); it != availableRobots.end(); ++it)
    {
        Robot_Ptr robot = *it;
        if (robot->GetStorageAvailable() <= 0)
        {
            int bestBinBot = GetBestBinBot(robot->GetID());
            if (bestBinBot != -1)
            {
                Dump_Ptr dp(new DumpWrapper());
                dp->SetRobot1(robot->GetID());
                dp->SetRobot2(bestBinBot);
                dp->SetPose1(Conversion::SetPose(0,1,1,0));
                dp->SetPose2(Conversion::SetPose(1,0,0,1));
                dp->SetTime(ros::Time::now());
                dp->SetStatus(TaskResult::INPROGRESS);
                m_tm.AddDump(dp);

                //set new robots' state
                m_robots[robot->GetID()]->SetState(RobotState::DUMPING);
                m_robots[bestBinBot]->SetState(RobotState::DUMPING);

                m_tm.SendDump(dp->GetID());
            }
        }
    }

    //Refresh the available robots list & goals list in case robots have been assigned to dump and/or been called off from a goal

    std::vector<Goal_Ptr> availableGoals = m_tm.GetAvailableGoals();
    availableRobots = GetAvailableRobots();
    for (std::vector<Goal_Ptr>::iterator it = availableGoals.begin(); it != availableGoals.end(); ++it)
    {
        Goal_Ptr gp = *it;
        // ROS_INFO_STREAM("Checking goal : "<<gp->GetID());
        int bestRobot = GetBestCollectorbot(gp->GetID());
        if (bestRobot != -1)
        {
            ROS_INFO_STREAM("Found a collector that can go to goal ("<<gp->GetID()<<") : "<<bestRobot);
            gp->SetRobot(bestRobot);
            gp->SetStatus(TaskResult::INPROGRESS);

            //set new robot's state
            m_robots[bestRobot]->SetState(RobotState::COLLECTING);

            m_tm.SendGoal(gp->GetID());
        }
    }
*/
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
    wp->SetStatus(TaskResult::INPROGRESS);

    // Assign robot to best waypoint
    m_robots[robot_id]->SetState(RobotState::NAVIGATING);
    m_tm.SendWaypoint(waypoint_id);

    // Track assignment in statistics
    // map<robot_id, map<waypoint_id, double_seconds_since_start> >
    robot_waypoint_times[robot_id].insert(std::pair<int,double> (waypoint_id, TimeSinceStart() ) );
    // robot_waypoint_times.insert(std::pair<int, std::map<int,double> >(robot_id,));

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
    std::vector<Robot_Ptr> availableRobots = GetAvailableRobots();

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
                ROS_ERROR_STREAM_THROTTLE(1, "Error Assigning Robot " << m_robots[bestRobot]->GetName() << "(" << m_robots[bestRobot]->GetID() << ")"
                                             << " - Waypoint(" << wp->GetID() << ")");
            }
        }
        else
        {
            // ROS_INFO_THROTTLE(1, "FAILED TO FIND A ROBOT");
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
    std::vector<Robot_Ptr> availableRobots = GetAvailableRobots();

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
                ROS_ERROR_STREAM_THROTTLE(1, "Error Assigning Robot " << m_robots[bestRobot]->GetName() << "(" << m_robots[bestRobot]->GetID() << ")"
                                             << " - Waypoint(" << wp->GetID() << ")");
            }
        }
        else
        {
            // ROS_INFO_THROTTLE(1, "FAILED TO FIND A ROBOT");
        }
    }
}

/***********************************************************************
 *  Method: GlobalPlanner::GetAvailableRobots
 *  Params:
 * Returns: std::vector<Robot_Ptr> of all available robots
 * Effects: Returns all robots that are in the available state
 ***********************************************************************/
std::vector<Robot_Ptr> GlobalPlanner::GetAvailableRobots()
{
    std::vector<Robot_Ptr> v;
    for (std::map<int, Robot_Ptr>::iterator it = m_robots.begin(); it != m_robots.end(); ++it)
    {
        // If it's in the "waiting" state, it's avilable for task setting
        if (it->second->GetState() == RobotState::WAITING)
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
    ROS_WARN("TODO: Getting FIRST instead of BEST");
    return GetFirstAvailableBot(RobotState::BIN_BOT);
}

/***********************************************************************
 *  Method: GlobalPlanner::GetBestCollectorbot
 *  Params: (int goalID)
 * Returns: int id of robot
 * Effects: Returns the id of the best collector bot for given goal id
 ***********************************************************************/
int GlobalPlanner::GetBestCollectorbot(int goalID)
{
    ROS_WARN("TODO: Getting FIRST instead of BEST");
    return GetFirstAvailableBot(RobotState::COLLECTOR_BOT);
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
 *  Method: GlobalPlanner::GetRobotClosestToWaypoint
 *  Params: int waypointID, GlobalPlanner::ROBOT_TYPE type
 * Returns: int id of robot
 * Effects: Returns the physically closest robot of the right type that is available (waiting and has storage space)
 ***********************************************************************/
int GlobalPlanner::GetRobotClosestToWaypoint(int waypointID, RobotState::Type type)
{
    std::map<int, Waypoint_Ptr> waypoints = m_tm.GetWaypoints();
    geometry_msgs::Pose waypoint_pose = waypoints[waypointID]->GetPose();

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
            double dist = Get2DPoseDistance(robot->GetPose(), waypoint_pose);

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
    SendSound("mario_coin.wav");
}


// System finished
void GlobalPlanner::Finished()
{
    SendSound("mario_1_up.wav");
    // TODO: Wrap up statistics here.
    ROS_INFO("Global Planner finished in : %g seconds", TimeSinceStart() );

    // Print out assignment table
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
}

// True if all waypoints chomped (success/failure/abort of any kind), false if any are still available or in progress
bool GlobalPlanner::isFinished() {
    return m_tm.isFinished();
}


// setup callbacks, regiser services, load waypoints...
bool GlobalPlanner::SetupCallbacks()
{
    m_robotSub = m_nh->subscribe("robot_status", 10, &GlobalPlanner::cb_robotStatus, this);
    m_eStopPub = m_nh->advertise<std_msgs::Empty>("e_stop_pub", 100);

    // Publisher to send current state to the rest of the world when transition happens
    m_soundPub = m_nh->advertise<global_planner::SoundMsg>("play_sound", 100);

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
        // m_robots[id]->SetData(status);
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

        ROS_INFO_STREAM("Added new robot: "<<ptr->ToString());
    }
}


/***********************************************************************
 *  Method: GlobalPlanner::SendSound
 *  Params: std::string filename, int num_times
 * Returns: void
 * Effects:	play the sound specified in the filename
 *  (relative to the 'resources/sounds' folder)
 *  for the number of timess specified
 ***********************************************************************/
void GlobalPlanner::SendSound(std::string filename, int num_times)
{
  global_planner::SoundMsg s;
  s.filename = filename;
  s.num_times = num_times;
  s.text_output = std::string();
  m_soundPub.publish(s);
}


/***********************************************************************
 *  Method: GlobalPlanner::SendSound
 *  Params: std::string filename
 * Returns: void
 * Effects:	play the sound specified in the filename
 *  (relaative to the 'resources/sounds' folder)
 ***********************************************************************/
void GlobalPlanner::SendSound(std::string filename)
{
    SendSound(filename, 1);
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
                ROS_INFO_STREAM_THROTTLE(10.0, "Received response from robot: "<<s.response.status.id);
                m_robots[s.request.id]->SetData(s.response.status);
            }
            else
            {
                ROS_ERROR_STREAM("Could not receive response from robot: "<<it->first);
            }
        }
        else
        {
            std::string serviceTopic = Conversion::RobotIDToServiceName(it->first);
            m_statusServices[it->first] = m_nh->serviceClient<global_planner::RobotStatusSrv>(serviceTopic, true);
            ROS_ERROR_STREAM("Not connected to robot: "<<it->first);
        }
    }
}


