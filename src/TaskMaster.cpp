#include "global_planner/TaskMaster.h"

/***********************************************************************
 *  Method: TaskMaster::TaskMaster
 *  Params:
 * Effects: Initialize pointers
 ***********************************************************************/
TaskMaster::TaskMaster()
{
    m_nh = (0);
}


/***********************************************************************
 *  Method: TaskMaster::Init
 *  Params: ros::NodeHandle &nh, std::map<int, Robot_Ptr> robots
 * Returns: bool
 * Effects: Initialize taskmaster with robots, nodehandles, and waypoints.
 ***********************************************************************/
bool TaskMaster::Init(ros::NodeHandle* nh, std::map<int, Robot_Ptr> robots, std::string waypoint_filename)
{
    m_nh = nh;

    Clear();

    UpdateRobotMap(robots);

    SetupTopics();
    RegisterServices();

    // Load waypoint file
    std::string path_to_waypoints = ros::package::getPath("global_planner");
    path_to_waypoints += std::string("/resources/waypoint_lists/");
    path_to_waypoints += waypoint_filename;
    LoadWaypoints(path_to_waypoints);

    ros::spinOnce();
}


/***********************************************************************
 *  Method: TaskMaster::updateRobotMap
 *  Params: std::map<int, Robot_Ptr> new_robots
 * Returns: void
 * Effects:
 ***********************************************************************/
void TaskMaster::UpdateRobotMap(std::map<int, Robot_Ptr> new_robots)
{
    m_robots = new_robots;
    RegisterServices();
}


/***********************************************************************
 *  Method: TaskMaster::SetupCallbacks
 *  Params:
 * Returns: bool
 * Effects: Setup topic subscribers and publishers
 ***********************************************************************/
bool TaskMaster::SetupTopics()
{
    ROS_INFO_STREAM("Setting up callback topics for Tasks");

    ROS_INFO_STREAM("Setting up subscribers for tasks");
    //Let's do something easier for now...
    m_goalSub = m_nh->subscribe("goal_finished", 10, &TaskMaster::cb_goalFinished, this);
    m_waypointSub = m_nh->subscribe("waypoint_finished", 10, &TaskMaster::cb_waypointFinished, this);
    m_dumpSub = m_nh->subscribe("dump_finished", 10, &TaskMaster::cb_dumpFinished, this);

    ROS_INFO_STREAM("Setting up subscriber for goals seen");
    m_goalSeenSub = m_nh->subscribe("goal_seen", 10, &TaskMaster::cb_goalSeen, this);

    ROS_INFO_STREAM("Finished Setting up subscribers");
}


/***********************************************************************
 *  Method: TaskMaster::RegisterServices
 *  Params:
 * Returns: bool
 * Effects: register any services (both setting up and for listening)
 ***********************************************************************/
bool TaskMaster::RegisterServices()
{
    for (std::map<int, Robot_Ptr>::iterator i = m_robots.begin(); i != m_robots.end(); ++i)
    {
        std::string waypointServiceTopic = Conversion::RobotIDToWaypointTopic(i->first);
        std::string goalServiceTopic = Conversion::RobotIDToGoalTopic(i->first);
        std::string dumpServiceTopic = Conversion::RobotIDToDumpTopic(i->first);

        m_waypointClients[i->first] = m_nh->serviceClient<global_planner::WaypointSrv>(waypointServiceTopic, true);
        m_dumpClients[i->first] = m_nh->serviceClient<global_planner::DumpSrv>(dumpServiceTopic, true);
        m_goalClients[i->first] = m_nh->serviceClient<global_planner::GoalSrv>(goalServiceTopic, true);
    }
}

/***********************************************************************
 *  Method: TaskMaster::LoadWaypoints
 *  Params: std::string filename
 * Returns: void
 * Effects: load waypoints from a file
 ***********************************************************************/
void TaskMaster::LoadWaypoints(std::string filename)
{
    ROS_INFO_STREAM("Loading waypoints from file: "<<filename);
    std::ifstream fin(filename.c_str());
    std::string s;
    //read a line into 's' from 'fin' each time
    for(int i=0; getline(fin,s); i++){
        //use the string 's' as input stream, the usage of 'sin' is just like 'cin'
        std::istringstream sin(s);
        double x,y,rz,rw;
        int id;
        sin>>id;
        sin>>x;
        sin>>y;
        sin>>rz;
        sin>>rw;
        Waypoint_Ptr wp(new WaypointWrapper(id, x, y, rz, rw));

        ROS_INFO_STREAM("Loaded waypoint["<<i<<"]: "<<x<<", "<<y<<", "<<rz<<", "<<rw);
        AddWaypoint(wp);
    }
}


/***********************************************************************
 *  Method: TaskMaster::AddGoal
 *  Params: Goal_Ptr goal
 * Returns: bool
 * Effects: add the goal to the list of goals
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
    	//TODO: verify that this works...
        m_goalMap[goal->GetID()] = goal;
    }
}


/***********************************************************************
 *  Method: TaskMaster::AddWaypoint
 *  Params: Waypoint_Ptr waypoint
 * Returns: bool
 * Effects: add the waypoint to the list of waypoints
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
 * Effects: add the dump to the list of dumps
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
 *  Method: TaskMaster::Clear
 *  Params:
 * Returns: bool
 * Effects: clear all lists
 ***********************************************************************/
bool TaskMaster::Clear()
{
    m_dumpMap.clear();
    m_waypointMap.clear();
    m_goalMap.clear();
    m_robots.clear();

    m_waypointClients.clear();
    m_goalClients.clear();
    m_dumpClients.clear();
}


/***********************************************************************
 *  Method: TaskMaster::SendWaypoint
 *  Params: int wpID
 * Returns: bool
 * Effects: Send the waypoint message specified by the parameter
 ***********************************************************************/
bool TaskMaster::SendWaypoint(int wpID)
{
    global_planner::WaypointMsg wpMsg = m_waypointMap[wpID]->GetMessage();
    int robotID = wpMsg.robotID;
    if (wpID != -1 && robotID >= 0)
    {
        global_planner::WaypointSrv s;
        s.request.msg = wpMsg;
        if (m_waypointClients[robotID].call(s))
        {
            if (s.response.result == 0)
            {
                return true;
            }
            else
            {
                ROS_ERROR_STREAM("Sending waypoint. Bad response (resp = "<<s.response.result<<") from robot: "<<robotID);
                return false;
            }
        }
        else
        {
            ROS_ERROR_STREAM("Failed to connect to robot["<<robotID);
            return false;
        }
    }
    else
    {
        ROS_ERROR_STREAM("Sending waypoint. invalid id's: wp = "<<wpID<<", robot = "<<robotID);
    }
    return false;
}


/***********************************************************************
 *  Method: TaskMaster::SendGoal
 *  Params: int goalID
 * Returns: bool
 * Effects: Send the goal message specified by the parameter
 ***********************************************************************/
bool TaskMaster::SendGoal(int goalID)
{
    global_planner::GoalMsg gm = m_goalMap[goalID]->GetMessage();

    int robotID = gm.robotID;
    if (goalID != -1 && robotID >= 0)
    {
        global_planner::GoalSrv s;
        s.request.msg = gm;
        if (m_goalClients[robotID].call(s))
        {
            if (s.response.result == 0)
            {
                return true;
            }
            else
            {
                ROS_ERROR_STREAM("Sending goal. Bad response (resp = "<<s.response.result<<") from robot: "<<robotID);
                return false;
            }
        }
        else
        {
            ROS_ERROR_STREAM("Failed to connect to robot["<<robotID);
            return false;
        }
    }
    else
    {
        ROS_ERROR_STREAM("Sending goal. invalid id's: goalID = "<<goalID<<", robot = "<<robotID);
    }
    return false;
}

// TODO : Replace senddump with same stuff from sendgoal
/***********************************************************************
 *  Method: TaskMaster::SendDump
 *  Params: int dumpID
 * Returns: bool
 * Effects: Send the dump message specified by the parameter
 ***********************************************************************/
bool TaskMaster::SendDump(int dumpID)
{
    global_planner::DumpMsg dm = m_dumpMap[dumpID]->GetMessage();
    if (dm.id != -1 && dm.robotID1 >= 0 && dm.robotID2 >= 0)
    {
        global_planner::DumpSrv s;
        s.request.msg = dm;
        bool call1 = false;
        bool call2 = false;
        bool response1 = false;
        bool response2 = false;

        call1 = m_dumpClients[dm.robotID1].call(s);
        if (call1)
        {
            response1 = s.response.result == 0;
            if (!response1) 
            {
                ROS_ERROR_STREAM("Service response 1 failed for dump id: " << 
                                 dm.id << ", Robot1(" << dm.robotID1 << 
                                 "), Robot2(" << dm.robotID2 << 
                                 "), response: " << s.response.result);
            }
        }
        else
        {
            ROS_ERROR_STREAM("Service call 1 failed for dump id: " << dm.id << 
                             ", Robot1(" << dm.robotID1 << "), Robot2(" << 
                             dm.robotID2 << ")");
        }
        
        call2 = m_dumpClients[dm.robotID2].call(s);
        if (call2)
        {
            response2 = s.response.result == 0;
            if (!response2) 
            {
                ROS_ERROR_STREAM("Service response 2 failed for dump id: " << 
                                 dm.id << ", Robot1(" << dm.robotID1 << 
                                 "), Robot2(" << dm.robotID2 << 
                                 "), response: " << s.response.result);
            }
        }
        else
        {
            ROS_ERROR_STREAM("Service call 2 failed for dump id: " << dm.id << 
                             ", Robot1(" << dm.robotID1 << "), Robot2(" << 
                             dm.robotID2 << ")");
        }

        // All passed
        return call1 && call2 && response1 && response2;
    }
    else {
        ROS_ERROR_STREAM("Invalid dump or robot ids: " << dm.id << ", Robot1(" << dm.robotID1 << "), Robot2(" << dm.robotID2 << ")");
    }
    return false;
}


/***********************************************************************
 *  Method: TaskMaster::GetGoals
 *  Params:
 * Returns: std::vector<Goal_Ptr>
 * Effects: Get all goals
 ***********************************************************************/
std::map<int, Goal_Ptr> TaskMaster::GetGoals()
{
    return m_goalMap;
}


/***********************************************************************
 *  Method: TaskMaster::GetWaypoints
 *  Params:
 * Returns: std::vector<Waypoint_Ptr>
 * Effects: Get all waypoints
 ***********************************************************************/
std::map<int, Waypoint_Ptr> TaskMaster::GetWaypoints()
{
    return m_waypointMap;
}


/***********************************************************************
 *  Method: TaskMaster::GetDumps
 *  Params:
 * Returns: std::vector<Dump_Ptr>
 * Effects: Get all dumps
 ***********************************************************************/
std::map<int, Dump_Ptr> TaskMaster::GetDumps()
{
    return m_dumpMap;
}


/***********************************************************************
 *  Method: TaskMaster::cb_goalFinished
 *  Params: const global_planner::GoalFinished::ConstPtr& msg
 * Returns: void
 * Effects: Callback for when a goal is finished
 ***********************************************************************/
void TaskMaster::cb_goalFinished(const global_planner::GoalFinished::ConstPtr& msg)
{
    int status = msg->status;
    m_goalMap[msg->id]->SetStatus(Conversion::IntToTaskResult(status));
    if (status == TaskResult::SUCCESS)
    {
        ROS_INFO_STREAM("Goal finished successfully");
    }
    else
    {
        ROS_ERROR_STREAM("ERROR, Goal finished with status: " << msg->status
                         << " : " << Conversion::TaskResultToString(Conversion::IntToTaskResult(msg->status)));
    }
}


/***********************************************************************
 *  Method: TaskMaster::cb_waypointFinished
 *  Params: const global_planner::WaypointFinished::ConstPtr& msg
 * Returns: void
 * Effects: callback for when a waypoint is finished
 ***********************************************************************/
void TaskMaster::cb_waypointFinished(const global_planner::WaypointFinished::ConstPtr& msg)
{
    int status = msg->status;
    m_waypointMap[msg->id]->SetStatus(Conversion::IntToTaskResult(status));
    if (status == TaskResult::SUCCESS)
    {
        ROS_INFO_STREAM("Waypoint["<<msg->id<<"] reached successfully");
    }
    else
    {
        ROS_ERROR_STREAM("ERROR, Waypoint finished with status: " << msg->status
                         << " : " << Conversion::TaskResultToString(Conversion::IntToTaskResult(msg->status)));
        //FOR NOW, let's just say it's available after a failure
        m_waypointMap[msg->id]->SetStatus(TaskResult::AVAILABLE);
    }
}


/***********************************************************************
 *  Method: TaskMaster::cb_dumpFinished
 *  Params: const global_planner::DumpFinished::ConstPtr& msg
 * Returns: void
 * Effects: callback for when a dump is finished
 ***********************************************************************/
void TaskMaster::cb_dumpFinished(const global_planner::DumpFinished::ConstPtr& msg)
{
    int status = msg->status;
    m_dumpMap[msg->id]->SetStatus(Conversion::IntToTaskResult(status));

    if (status == TaskResult::SUCCESS)
    {
        ROS_INFO_STREAM("Dumping was successful");
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
 * Effects: callback for when a goal is seen by an april tag manager
 ***********************************************************************/
void TaskMaster::cb_goalSeen(const global_planner::GoalSeen::ConstPtr &msg)
{
    int goalID = msg->id;

    ROS_INFO_STREAM("Goal Seen: "<<goalID);

    std::map<int,Goal_Ptr>::iterator it = m_goalMap.find(goalID);

    // it is already in the map... update?
    if(it != m_goalMap.end())
    {
        Goal_Ptr currentGoal = m_goalMap[goalID];
        if (currentGoal->GetCompleted() == true)
        {
            return;
        }

        ros::Time time = msg->time;
        geometry_msgs::Pose pose = msg->pose;

        if ( time >= currentGoal->GetTime() )
        {
            //Newer information... Update Pose if they are off by more than a small distance
            //For now, just assume we update it every time. If it is currently assigned to a bot, tell the bot of the new location
            if (true)
            {
                //update the goal's values
                currentGoal->SetTime(time);
                currentGoal->SetPose(pose);

                SendGoal(goalID);
            }
        }
    }
    else //Not yet in the map
    {
        ros::Time time = msg->time;
        geometry_msgs::Pose pose = msg->pose;

        Goal_Ptr ptr(new GoalWrapper());
        ptr->SetPose(pose);
        ptr->SetID(goalID);
        ptr->SetTime(time);
        ptr->SetStatus(TaskResult::AVAILABLE);
        ptr->SetRobot(-1);
        ROS_INFO_STREAM("Creating new goal in map: "<<goalID<<" : "<<ptr->ToString());
        ROS_INFO_STREAM("Adding to map");
        m_goalMap[goalID] = ptr;
    }
}


/***********************************************************************
 *  Method: TaskMaster::GetAvailableGoals
 *  Params:
 * Returns: std::vector<Goal_Ptr>
 * Effects: Get all available goals
 ***********************************************************************/
std::vector<Goal_Ptr> TaskMaster::GetAvailableGoals()
{
    std::vector<Goal_Ptr> v;
    for (std::map<int, Goal_Ptr>::iterator it = m_goalMap.begin(); it != m_goalMap.end(); ++it)
    {
        // If it's avilable for task setting
        if (it->second->GetStatus() == TaskResult::AVAILABLE)
        {
            v.push_back(it->second);
        }
    }
    return v;
}


/***********************************************************************
 *  Method: TaskMaster::GetAvailableWaypoints
 *  Params:
 * Returns: std::vector<Waypoint_Ptr>
 * Effects: Get all available waypoints
 ***********************************************************************/
std::vector<Waypoint_Ptr> TaskMaster::GetAvailableWaypoints()
{
    std::vector<Waypoint_Ptr> v;
    for (std::map<int, Waypoint_Ptr>::iterator it = m_waypointMap.begin(); it != m_waypointMap.end(); ++it)
    {
        // If it's avilable for task setting
        if (it->second->GetStatus() == TaskResult::AVAILABLE)
        {
            v.push_back(it->second);
        }
    }
    return v;
}


/***********************************************************************
 *  Method: TaskMaster::isFinished()
 *  Params:
 * Returns: bool whether all waypoints have reached an end state
 * Effects: Get whether all waypoints have reached an end state
 ***********************************************************************/
bool TaskMaster::isFinished()
{
    for (std::map<int, Waypoint_Ptr>::iterator it = m_waypointMap.begin(); it != m_waypointMap.end(); ++it)
    {
        // If it's avilable for task setting
        if (it->second->GetStatus() == TaskResult::AVAILABLE || it->second->GetStatus() == TaskResult::INPROGRESS)
        {
            return false;
        }
    }
    return true;
}


/***********************************************************************
 *  Method: TaskMaster::GetAvailableDumps
 *  Params:
 * Returns: std::vector<Dump_Ptr>
 * Effects: get all available dumps
 ***********************************************************************/
std::vector<Dump_Ptr> TaskMaster::GetAvailableDumps()
{
    std::vector<Dump_Ptr> v;
    for (std::map<int, Dump_Ptr>::iterator it = m_dumpMap.begin(); it != m_dumpMap.end(); ++it)
    {
        // If it's avilable for task setting
        if (it->second->GetStatus() == TaskResult::AVAILABLE)
        {
            v.push_back(it->second);
        }
    }
    return v;
}


