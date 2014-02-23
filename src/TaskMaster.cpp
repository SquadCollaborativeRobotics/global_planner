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
 *  Params: int goalID, ResultStatus status
 * Returns: void
 * Effects:
 ***********************************************************************/
void TaskMaster::cb_goalFinished(int goalID, ResultStatus status)
{
}


/***********************************************************************
 *  Method: TaskMaster::cb_waypointFinished
 *  Params: int wpID, ResultStatus status
 * Returns: void
 * Effects:
 ***********************************************************************/
void TaskMaster::cb_waypointFinished(int wpID, ResultStatus status)
{
}


/***********************************************************************
 *  Method: TaskMaster::cb_dumpFinished
 *  Params: int dumpID, ResultStatus status
 * Returns: void
 * Effects:
 ***********************************************************************/
void TaskMaster::cb_dumpFinished(int dumpID, ResultStatus status)
{
}


/***********************************************************************
 *  Method: TaskMaster::SetupCallbacks
 *  Params:
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool TaskMaster::SetupCallbacks()
{
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


