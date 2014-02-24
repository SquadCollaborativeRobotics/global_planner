#include <global_planner/GlobalPlanner.h>

GlobalPlanner::GlobalPlanner()
{
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
}

// Executive function
void GlobalPlanner::Execute()
{
    static int count = 0;
    ROS_INFO_THROTTLE(5,"Executive");

    ros::spinOnce();

    if (count % 50 == 0)
    {
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
    count++;

    // Get Robot Status...


    // FOR each pair of robots that're just chillin in a dump stage and are stopped near the handoff location
    //      Check if robot1 & robot2 are within handoff threshold
    //      Send handoff message to each robot giving them their new capacities
    //
    // IF robot in wait state AND is full AND there are goals
    //      Find the best dump location available to the robot
    //      Update Dump List -> Send Dump Message
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
        if (it->second->GetState() != RobotState::WAITING)
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
}

bool GlobalPlanner::RegisterServices()
{
    //register services
    for (int i=0; i<m_robots.size(); i++)
    {

    }
}

// get robot information
void GlobalPlanner::cb_robotStatus(const global_planner::RobotStatus::ConstPtr& msg)
{
    int id = msg->id;
    global_planner::RobotStatus status = *msg;
    m_robots[id]->SetData(status);
}

// Send request to all listening robots that
// they should send out their information to be added to the list of bots
int GlobalPlanner::FindRobots()
{
    std::vector< std::string > names;
    names.push_back(std::string("collector1"));
    names.push_back(std::string("collector2"));
    names.push_back(std::string("bin1"));

    m_robots.clear();

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

    return names.size();
}
