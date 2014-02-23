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
bool TaskMaster::Init(ros::NodeHandle &nh, std::map<int, Robot_Ptr> robots)
{
    m_robots = robots;

    SetupTopics();
}


/***********************************************************************
 *  Method: TaskMaster::AddGoal
 *  Params: boost::shared_ptr<Goal> goal
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool TaskMaster::AddGoal(boost::shared_ptr<Goal> goal)
{
}


/***********************************************************************
 *  Method: TaskMaster::AddWaypoint
 *  Params: boost::shared_ptr<Waypoint> waypoint
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool TaskMaster::AddWaypoint(boost::shared_ptr<Waypoint> waypoint)
{
}


/***********************************************************************
 *  Method: TaskMaster::AddDump
 *  Params: boost::shared_ptr<Dump> dump
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool TaskMaster::AddDump(boost::shared_ptr<Dump> dump)
{
}


/***********************************************************************
 *  Method: TaskMaster::GetGoal
 *  Params: int goalID
 * Returns: Goal_Ptr
 * Effects:
 ***********************************************************************/
Goal_Ptr TaskMaster::GetGoal(int goalID)
{
}


/***********************************************************************
 *  Method: TaskMaster::GetWaypoint
 *  Params: int wpID
 * Returns: Waypoint_Ptr
 * Effects:
 ***********************************************************************/
Waypoint_Ptr TaskMaster::GetWaypoint(int wpID)
{
}


/***********************************************************************
 *  Method: TaskMaster::GetDump
 *  Params: int dumpID
 * Returns: Dump_Ptr
 * Effects:
 ***********************************************************************/
Dump_Ptr TaskMaster::GetDump(int dumpID)
{
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
 *  Method: TaskMaster::RemoveGoal
 *  Params: int goalID
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool TaskMaster::RemoveGoal(int goalID)
{
}


/***********************************************************************
 *  Method: TaskMaster::RemoveWaypoint
 *  Params: int wpID
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool TaskMaster::RemoveWaypoint(int wpID)
{
}


/***********************************************************************
 *  Method: TaskMaster::RemoveDump
 *  Params: int dumpID
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool TaskMaster::RemoveDump(int dumpID)
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
}


/***********************************************************************
 *  Method: TaskMaster::SendWaypoint
 *  Params: int wpID
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool TaskMaster::SendWaypoint(int wpID)
{
}


/***********************************************************************
 *  Method: TaskMaster::SendGoal
 *  Params: int goalID
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool TaskMaster::SendGoal(int goalID)
{
}


/***********************************************************************
 *  Method: TaskMaster::SendDump
 *  Params: int dumpID
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool TaskMaster::SendDump(int dumpID)
{
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
    if (status == TaskResult::SUCCESS)
    {
        m_goalMap.erase(msg->id);
    }
    else
    {
        ROS_ERROR_STREAM("ERROR, Goal finished with status: "<<msg->status);
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
    if (status == TaskResult::SUCCESS)
    {
        m_waypointMap.erase(msg->id);
    }
    else
    {
        ROS_ERROR_STREAM("ERROR, Goal finished with status: "<<msg->status<<" : "<<Conversion::ReturnStatusToString(Conversion::ReturnIntToStatus(msg->status)));
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
    if (status == TaskResult::SUCCESS)
    {
        m_dumpMap.erase(msg->id);
    }
    else
    {
        ROS_ERROR_STREAM("ERROR, Dump finished with status: "<<msg->status);
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
    m_dumpPub = m_nh->advertise<global_planner::DumpMsg>("dump_pub", 100);
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


