#include <global_planner/GlobalPlanner.h>

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
    RegisterServices();

    FindRobots();

    ROS_INFO_STREAM("Initializing TaskMaster");
    m_tm.Init(nh, m_robots, "testList1.points");

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
    std::vector<Robot_Ptr> availableRobots = GetAvailableRobots();

    //TODO: Handoff

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
    }

    ros::spinOnce();
}


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


int GlobalPlanner::GetBestBinBot(int idOfRobotThatNeedsIt)
{
    for (std::map<int, Robot_Ptr>::iterator it = m_robots.begin(); it != m_robots.end(); ++it)
    {
        //if it is a binbot
        if (it->second->GetType() == false)
        {
            if (it->second->GetState() == RobotState::WAITING)
            {
                if (it->second->GetStorageAvailable() > 0)
                {
                    return it->first;
                }
            }
        }
    }
    return -1;
}


int GlobalPlanner::GetBestCollectorbot(int goalID)
{
    std::map<int, Goal_Ptr> goals = m_tm.GetGoals();
    geometry_msgs::Pose goalPose = goals[goalID]->GetPose();

    for (std::map<int, Robot_Ptr>::iterator it = m_robots.begin(); it != m_robots.end(); ++it)
    {
        //if it is a collectorbot
        if (it->second->GetType() == true)
        {
            if (it->second->GetState() == RobotState::WAITING)
            {
                if (it->second->GetStorageAvailable() > 0)
                {
                    return it->first;
                }
            }
        }
    }
    return -1;
}


int GlobalPlanner::GetBestSearchBot(int waypointID)
{
    std::map<int, Waypoint_Ptr> waypoints = m_tm.GetWaypoints();
    geometry_msgs::Pose pose = waypoints[waypointID]->GetPose();

    for (std::map<int, Robot_Ptr>::iterator it = m_robots.begin(); it != m_robots.end(); ++it)
    {
        if (it->second->GetState() == RobotState::WAITING)
        {
            return it->first;
        }
    }
    return -1;
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

bool GlobalPlanner::RegisterServices()
{
}

// get robot information
void GlobalPlanner::cb_robotStatus(const global_planner::RobotStatus::ConstPtr& msg)
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

// Send request to all listening robots that
// they should send out their information to be added to the list of bots
int GlobalPlanner::FindRobots()
{
    std::vector< std::string > names;
    names.push_back(std::string("collector1"));

    m_robots.clear();

    /*
    for (int i=0; i<names.size(); i++)
    {
        Robot_Ptr robot (new RobotStatusWrapper());
        int id = i;
        robot->SetName(names[i]);
        robot->SetID(id);
        robot->SetStorageUsed(0);
        robot->SetStorageCapacity(3);
        m_robots[id] = robot;

        if (names[i].compare("bin1") == 0)
        {
            m_robots[id]->SetType(false);
        }
        else
        {
            m_robots[id]->SetType(true);
        }
    }
    */

    return names.size();
}
/***********************************************************************
 *  Method: GlobalPlanner::SendSound
 *  Params: std::string filename, int num_times
 * Returns: void
 * Effects:
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
 * Effects:
 ***********************************************************************/
void GlobalPlanner::SendSound(std::string filename)
{
    SendSound(filename, 1);
}

