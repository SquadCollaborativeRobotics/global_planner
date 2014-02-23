#include "global_planner/TaskMaster.h"

/***********************************************************************
 *  Method: TaskMaster::TaskMaster
 *  Params:
 * Effects:
 ***********************************************************************/
TaskMaster::TaskMaster()
{
}


/***********************************************************************
 *  Method: TaskMaster::Init
 *  Params: ros::NodeHandle &nh, std::map<int, Robot_Ptr> robots
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool TaskMaster::Init(ros::NodeHandle* nh, std::map<int, Robot_Ptr> robots, std::string waypoint_filename)
{
    Clear();

    m_robots = robots;

    SetupTopics();
    RegisterServices();


    std::string path_to_waypoints = std::string("aplay -q ");
    path_to_waypoints += ros::package::getPath("global_planner");
    path_to_waypoints += std::string("/resources/waypoint_lists/");
    path_to_waypoints += waypoint_filename;

    LoadWaypoints(path_to_waypoints);
}


/***********************************************************************
 *  Method: TaskMaster::AddGoal
 *  Params: Goal_Ptr goal
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool TaskMaster::AddGoal(Goal_Ptr goal)
{
    std::map<int,Goal_Ptr>::iterator it = m_goalMap.find(goal->GetID());
    //If it is already in the map...
    if(it != m_goalMap.end())
    {
        ROS_ERROR_STREAM("Trying to add to the goal list when an entry already exists for key: "<<goal->GetID());
    }
    else
    {
        m_goalMap[goal->GetID()] = goal;
    }
}


/***********************************************************************
 *  Method: TaskMaster::AddWaypoint
 *  Params: Waypoint_Ptr waypoint
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool TaskMaster::AddWaypoint(Waypoint_Ptr waypoint)
{
    std::map<int,Waypoint_Ptr>::iterator it = m_waypointMap.find(waypoint->GetID());
    //If it is already in the map...
    if(it != m_waypointMap.end())
    {
        ROS_ERROR_STREAM("Trying to add to the waypoint list when an entry already exists for key: "<<waypoint->GetID());
    }
    else
    {
        m_waypointMap[waypoint->GetID()] = waypoint;
    }
}


/***********************************************************************
 *  Method: TaskMaster::AddDump
 *  Params: Dump_Ptr dump
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool TaskMaster::AddDump(Dump_Ptr dump)
{
    std::map<int,Dump_Ptr>::iterator it = m_dumpMap.find(dump->GetID());
    //If it is already in the map...
    if(it != m_dumpMap.end())
    {
        ROS_ERROR_STREAM("Trying to add to the dump list when an entry already exists for key: "<<dump->GetID());
    }
    else
    {
        m_dumpMap[dump->GetID()] = dump;
    }
}


/***********************************************************************
 *  Method: TaskMaster::UpdateGoal
 *  Params: int goalID, int robotID, geometry_msgs::PoseStamped pose
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool TaskMaster::UpdateGoal(int goalID, int robotID, geometry_msgs::PoseStamped pose)
{
}


/***********************************************************************
 *  Method: TaskMaster::UpdateWaypoint
 *  Params: int wpID, int robotID, geometry_msgs::PoseStamped pose
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool TaskMaster::UpdateWaypoint(int wpID, int robotID, geometry_msgs::PoseStamped pose)
{
}


/***********************************************************************
 *  Method: TaskMaster::UpdateDump
 *  Params: int dumpID, int robotID1, int robotID2, geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool TaskMaster::UpdateDump(int dumpID, int robotID1, int robotID2, geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2)
{
}


/***********************************************************************
 *  Method: TaskMaster::Clear
 *  Params:
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool TaskMaster::Clear()
{
    m_dumpMap.clear();
    m_waypointMap.clear();
    m_goalMap.clear();
    m_robots.clear();
}


/***********************************************************************
 *  Method: TaskMaster::SendWaypoint
 *  Params: int wpID
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool TaskMaster::SendWaypoint(int wpID)
{
    global_planner::WaypointMsg wpm = m_waypointMap[wpID]->GetMessage();
    m_waypointPub.publish(wpm);
}


/***********************************************************************
 *  Method: TaskMaster::SendGoal
 *  Params: int goalID
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool TaskMaster::SendGoal(int goalID)
{
    global_planner::GoalMsg gm = m_goalMap[goalID]->GetMessage();
    m_goalPub.publish(gm);
}


/***********************************************************************
 *  Method: TaskMaster::SendDump
 *  Params: int dumpID
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool TaskMaster::SendDump(int dumpID)
{
    global_planner::DumpMsg dm = m_dumpMap[dumpID]->GetMessage();
    m_dumpPub.publish(dm);
}


/***********************************************************************
 *  Method: TaskMaster::GetGoalList
 *  Params:
 * Returns: std::vector<Goal_Ptr>
 * Effects:
 ***********************************************************************/
std::vector<Goal_Ptr> TaskMaster::GetGoalList()
{
}


/***********************************************************************
 *  Method: TaskMaster::GetWaypointList
 *  Params:
 * Returns: std::vector<Waypoint_Ptr>
 * Effects:
 ***********************************************************************/
std::vector<Waypoint_Ptr> TaskMaster::GetWaypointList()
{
}


/***********************************************************************
 *  Method: TaskMaster::GetDumpList
 *  Params:
 * Returns: std::vector<Dump_Ptr>
 * Effects:
 ***********************************************************************/
std::vector<Dump_Ptr> TaskMaster::GetDumpList()
{
}


/***********************************************************************
 *  Method: TaskMaster::cb_goalFinished
 *  Params: const global_planner::GoalFinished::ConstPtr& msg
 * Returns: void
 * Effects:
 ***********************************************************************/
void TaskMaster::cb_goalFinished(const global_planner::GoalFinished::ConstPtr& msg)
{
    int status = msg->status;
    m_goalMap[msg->id]->SetStatus(Conversion::IntToTaskResult(status));
    if (status == TaskResult::SUCCESS)
    {
        //AWESOME
        ROS_INFO_STREAM("Goal finished successfully... AWESOME!!!");
    }
    else
    {
        ROS_ERROR_STREAM("ERROR, Goal finished with status: "<<msg->status<<" : "<<Conversion::TaskResultToString(Conversion::IntToTaskResult(msg->status)));
    }
}


/***********************************************************************
 *  Method: TaskMaster::cb_waypointFinished
 *  Params: const global_planner::WaypointFinished::ConstPtr& msg
 * Returns: void
 * Effects:
 ***********************************************************************/
void TaskMaster::cb_waypointFinished(const global_planner::WaypointFinished::ConstPtr& msg)
{
    int status = msg->status;
    m_waypointMap[msg->id]->SetStatus(Conversion::IntToTaskResult(status));
    if (status == TaskResult::SUCCESS)
    {
        ROS_INFO_STREAM("Waypoint reached successfully... AWESOME!!!");
    }
    else
    {
        ROS_ERROR_STREAM("ERROR, Waypoint finished with status: "<<msg->status<<" : "<<Conversion::TaskResultToString(Conversion::IntToTaskResult(msg->status)));
    }
}


/***********************************************************************
 *  Method: TaskMaster::cb_dumpFinished
 *  Params: const global_planner::DumpFinished::ConstPtr& msg
 * Returns: void
 * Effects:
 ***********************************************************************/
void TaskMaster::cb_dumpFinished(const global_planner::DumpFinished::ConstPtr& msg)
{
    int status = msg->status;
    m_dumpMap[msg->id]->SetStatus(Conversion::IntToTaskResult(status));

    if (status == TaskResult::SUCCESS)
    {
        ROS_INFO_STREAM("Dumping was successful... AWESOME!!!");
    }
    else
    {
        ROS_ERROR_STREAM("ERROR, Dump finished with status: "<<msg->status<<" : "<<Conversion::TaskResultToString(Conversion::IntToTaskResult(msg->status)));
    }
}


/***********************************************************************
 *  Method: TaskMaster::cb_goalSeen
 *  Params: const global_planner::GoalSeen::ConstPtr &msg
 * Returns: void
 * Effects:
 ***********************************************************************/
void TaskMaster::cb_goalSeen(const global_planner::GoalSeen::ConstPtr &msg)
{
    int goalID = msg->id;

    Goal_Ptr currentGoal = m_goalMap[goalID];
    if (currentGoal->GetCompleted() == true)
    {
        return;
    }

    ros::Time time = msg->time;
    geometry_msgs::Pose pose = msg->pose;

    if ( time > currentGoal->GetTime() )
    {
        //Newer information... Update Pose if they are off by more than a small distance
        //For now, just assume we update it every time. If it is currently assigned to a bot, tell the bot of the new location
        if (true)
        {
            if ( m_goalMap[goalID]->GetInProgress() == true )
            {
                //Re-Send goal
                SendGoal(goalID);
            }
        }
    }
}


/***********************************************************************
 *  Method: TaskMaster::SetupCallbacks
 *  Params:
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool TaskMaster::SetupTopics()
{
    /*
    std::stringstream topicName;
    for (std::map<int, Robot_Ptr>::iterator i = m_robots.begin(); i != m_robots.end(); ++i)
    {
        int id = i->first;
        std::string baseName = "/";
        baseName += m_robots[id]->GetName();
        baseName += "/";

        //create listener for finished statuses
        std::string topic = baseName + std::string("goal_finished");
        m_goalSubs[id] = m_nh->subscribe(topic, 10, &TaskMaster::cb_goalFinished, this);
        topic = baseName + std::string("waypoint_finished");
        m_waypointSubs[id] = m_nh->subscribe(topic, 10, &TaskMaster::cb_waypointFinished, this);
        topic = baseName + std::string("dump_finished");
        m_dumpSubs[id] = m_nh->subscribe(topic, 10, &TaskMaster::cb_dumpFinished, this);

        //create publisher for finished statuses
        topic = baseName + std::string("goal_finished");
        m_goalPubs[id] = m_nh->advertise<global_planner::GoalMsg>(topic, 10);
        topic = baseName + std::string("waypoint_finished");
        m_waypointPubs[id] = m_nh->advertise<global_planner::WaypointMsg>(topic, 10);
        topic = baseName + std::string("dump_finished");
        m_dumpPubs[id] = m_nh->advertise<global_planner::DumpMsg>(topic, 10);
    }
    */

    //Let's do something easier for now...
    m_goalSub = m_nh->subscribe("goal_finished", 10, &TaskMaster::cb_goalFinished, this);
    m_waypointSub = m_nh->subscribe("waypoint_finished", 10, &TaskMaster::cb_waypointFinished, this);
    m_dumpSub = m_nh->subscribe("dump_finished", 10, &TaskMaster::cb_dumpFinished, this);

    m_goalPub = m_nh->advertise<global_planner::GoalMsg>("goal_pub", 100);
    m_waypointPub = m_nh->advertise<global_planner::WaypointMsg>("waypoint_pub", 100);
    m_dumpPub = m_nh->advertise<global_planner::DumpMsg>("dump_pub", 10);

    m_goalSeenSub = m_nh->subscribe("goal_seen", 10, &TaskMaster::cb_goalSeen, this);
}


/***********************************************************************
 *  Method: TaskMaster::RegisterServices
 *  Params:
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool TaskMaster::RegisterServices()
{
}

/***********************************************************************
 *  Method: TaskMaster::LoadWaypoints
 *  Params: std::string filename
 * Returns: void
 * Effects:
 ***********************************************************************/
void TaskMaster::LoadWaypoints(std::string filename)
{
    std::ifstream fin(filename.c_str());
    std::string s;
    //read a line into 's' from 'fin' each time
    for(int i=0; getline(fin,s); i++){
        //use the string 's' as input stream, the usage of 'sin' is just like 'cin'
        std::istringstream sin(s);
        double x,y,z,w;
        int id;
        sin>>id;
        sin>>x;
        sin>>y;
        sin>>z;
        sin>>w;
        Waypoint_Ptr wp(new WaypointWrapper(id, x,y,z,w));

        ROS_INFO_STREAM("Loaded waypoint["<<i<<"]: "<<x<<", "<<y<<", "<<z<<", "<<w);
    }
}


