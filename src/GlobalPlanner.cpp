#include <global_planner/GlobalPlanner.h>
#include <math.h> /* sqrt */

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
        ROS_INFO_STREAM("Read from file: "<<waypointFile);

    m_tm.Init(nh, m_robots, waypointFile);

    ros::spinOnce();

    return true;
}

void GlobalPlanner::Display()
{
    m_lastDisplay = ros::Time::now();
    ROS_INFO_STREAM("Showing robots...");
    for (std::map<int, Robot_Ptr>::iterator it = m_robots.begin(); it != m_robots.end(); ++it)
    {
        ROS_INFO_STREAM(it->second->ToString());
    }

    ROS_INFO_STREAM("Showing waypoints...");
    std::map<int, Waypoint_Ptr> wps = m_tm.GetWaypoints();
    for (std::map<int, Waypoint_Ptr>::iterator it = wps.begin(); it != wps.end(); ++it)
    {
        ROS_INFO_STREAM(it->second->ToString());
    }

    ROS_INFO_STREAM("Showing goals...");
    std::map<int, Goal_Ptr> goals = m_tm.GetGoals();
    for (std::map<int, Goal_Ptr>::iterator it = goals.begin(); it != goals.end(); ++it)
    {
        ROS_INFO_STREAM(it->second->ToString());
    }

    ROS_INFO_STREAM("Showing Dumps...");
    std::map<int, Dump_Ptr> dumps = m_tm.GetDumps();
    for (std::map<int, Dump_Ptr>::iterator it = dumps.begin(); it != dumps.end(); ++it)
    {
        ROS_INFO_STREAM(it->second->ToString());
    }
}

// Executive function
void GlobalPlanner::Execute()
{
    ROS_INFO_THROTTLE(5,"Executive");

    ros::spinOnce();

    if ((ros::Time::now() - m_lastDisplay) > ros::Duration(3))
    {
        Display();
    }

    // Get Robot Status...

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

    boost::mutex::scoped_lock lock(m_robotMutex);

    std::vector<Robot_Ptr> availableRobots = GetAvailableRobots();

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
    std::vector<Waypoint_Ptr> availableWaypoints = m_tm.GetAvailableWaypoints();
    availableRobots = GetAvailableRobots();

    for (std::vector<Waypoint_Ptr>::iterator it = availableWaypoints.begin(); it != availableWaypoints.end(); ++it)
    {
        Waypoint_Ptr wp = *it;
        int bestRobot = GetBestSearchBot(wp->GetID());
        if (bestRobot != -1)
        {
            ROS_INFO_STREAM("Found a robot to explore waypoint ("<<wp->GetID()<<") : "<<bestRobot);
            wp->SetRobot(bestRobot);
            wp->SetStatus(TaskResult::INPROGRESS);

            //set new robot's state
            m_robots[bestRobot]->SetState(RobotState::NAVIGATING);
            m_tm.SendWaypoint(wp->GetID());
        }
        else
        {
            // ROS_INFO_THROTTLE(1, "FAILED TO FIND A ROBOT");
        }
    }
}

// Returns all robots that are in the available state
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

// Returns the best binbot for dumping with the robot specified in the param
int GlobalPlanner::GetBestBinBot(int idOfRobotThatNeedsIt)
{
    return GetFirstAvailableBot(RobotState::BIN_BOT);
}

// Returns the id of the best collector bot for given goal id
int GlobalPlanner::GetBestCollectorbot(int goalID)
{
    return GetFirstAvailableBot(RobotState::COLLECTOR_BOT);
}

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

// System finished
void GlobalPlanner::Finished()
{
    ROS_INFO("Finished");
    exit(0);
}


// setup callbacks, regiser services, load waypoints...
bool GlobalPlanner::SetupCallbacks()
{
    /*
    for (std::map<int, Robot_Ptr >::iterator it=m_robots.begin(); it!=m_robots.end(); ++it)
    {
        std::string robotStatusTopic = "/";
        robotStatusTopic += it->second->GetName();
        robotStatusTopic += "/status";
        ros::Subscriber sub = m_nh->subscribe(robotStatusTopic, 10, &GlobalPlanner::cb_robotStatus, this);
    }
    */

    m_robotSub = m_nh->subscribe("robot_status", 10, &GlobalPlanner::cb_robotStatus, this);
    m_eStopPub = m_nh->advertise<std_msgs::Empty>("e_stop_pub", 100);

    // Publisher to send current state to the rest of the world when transition happens
    m_soundPub = m_nh->advertise<global_planner::SoundMsg>("play_sound", 100);

    ros::spinOnce();
}


// Get robot information TODO: better definition
void GlobalPlanner::cb_robotStatus(const global_planner::RobotStatus::ConstPtr& msg)
{
    if (m_robotMutex.try_lock())
    {
        int id = msg->id;
        // ROS_INFO_STREAM_THROTTLE(1, "Received robot status: "<<id);
        global_planner::RobotStatus status = *msg;

        std::map<int, Robot_Ptr>::iterator it = m_robots.find(id);
        //If it is already in the map...
        if(it != m_robots.end())
        {
            m_robots[id]->SetData(status);
        }
        else
        {
            Robot_Ptr ptr(new RobotStatusWrapper());
            ptr->SetData(status);
            m_robots[id] = ptr;

            ROS_INFO_STREAM("Added new robot: "<<ptr->ToString());
        }
    }
    m_robotMutex.unlock();
}


/***********************************************************************
 *  Method: GlobalPlanner::SendSound
 *  Params: std::string filename, int num_times
 * Returns: void
 * Effects:	play the sound specified in the filename
 *  (relaative to the 'resources/sounds' folder)
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