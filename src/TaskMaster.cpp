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
bool TaskMaster::Init(ros::NodeHandle* nh, std::string waypoint_filename)
{
    m_nh = nh;

    Clear();

    SetupTopics();

    // Load waypoint file
    std::string path_to_waypoints = ros::package::getPath("global_planner");
    path_to_waypoints += std::string("/resources/waypoint_lists/");
    path_to_waypoints += waypoint_filename;
    LoadWaypoints(path_to_waypoints);

    ros::spinOnce();
}


bool TaskMaster::AdvertiseServices(int robotID)
{
    m_wpFinishedService[robotID] = m_nh->advertiseService(Conversion::RobotIDToWaypointFinishedTopic(robotID), &TaskMaster::cb_waypointFinished, this);
    m_dumpFinishedService[robotID] = m_nh->advertiseService(Conversion::RobotIDToDumpFinishedTopic(robotID), &TaskMaster::cb_dumpFinished, this);
    ros::spinOnce();
    ROS_INFO_STREAM("Advertised Taskmaster services for robot: "<<robotID);
    return true;
}

bool TaskMaster::RegisterClients(int robotID)
{
    bool waypointSetup = false;

    while ( waypointSetup == false )
    {
        ros::spinOnce();
        ROS_INFO_STREAM("Waiting 2 seconds for waypoint service to come up");
        bool success = ros::service::waitForService(Conversion::RobotIDToWaypointTopic(robotID), ros::Duration(2));
        if (success)
        {
            m_waypointClients[robotID] = m_nh->serviceClient<global_planner::WaypointSrv>(Conversion::RobotIDToWaypointTopic(robotID), true);
            if (m_waypointClients[robotID].isValid())
            {
                ROS_INFO_STREAM("Waypoint finished listening service successfully setup for robot: "<<robotID);
                waypointSetup = true;
            }
            else
            {
                ROS_ERROR_STREAM("Waypoint finished listening service FAILED to set up for robot: "<<robotID);
            }
        }
        else
        {
            ROS_ERROR_STREAM("waypoint waitForService timeout occured for robot: "<<robotID);
        }
    }

    bool dumpSetup = false;
    while ( dumpSetup == false )
    {
        ros::spinOnce();
        ROS_INFO_STREAM("Waiting 2 seconds for Dump service to come up");
        bool success = ros::service::waitForService(Conversion::RobotIDToDumpTopic(robotID), ros::Duration(2));
        if (success)
        {
            m_dumpClients[robotID] = m_nh->serviceClient<global_planner::DumpSrv>(Conversion::RobotIDToDumpTopic(robotID), true);
            if (m_dumpClients[robotID].isValid())
            {
                ROS_INFO_STREAM("Dump Finished listening service successfully setup for robot: "<<robotID);
                dumpSetup = true;
            }
            else
            {
                ROS_ERROR_STREAM("Dump Finished listening service FAILED to set up for robot: "<<robotID);
            }
        }
        else
        {
            ROS_ERROR_STREAM("dump waitForService timeout occured for robot: "<<robotID);
        }
    }
    ROS_INFO_STREAM("Registered services for robot: "<<robotID);
    return true;
}

/***********************************************************************
 *  Method: TaskMaster::SetupCallbacks
 *  Params:
 * Returns: bool
 * Effects: Setup topic subscribers and publishers
 ***********************************************************************/
bool TaskMaster::SetupTopics()
{
    ROS_INFO_STREAM("Setting up subscriber for goals seen");
    m_goalSeenSub = m_nh->subscribe("/goal_seen", 10, &TaskMaster::cb_goalSeen, this);

    ROS_INFO_STREAM("Finished Setting up subscribers");

    // Publishers to send human interface output
    m_soundPub = m_nh->advertise<std_msgs::String>("/interface_sound", 100);
    m_textPub = m_nh->advertise<std_msgs::String>("/interface_text", 100);
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
    for(int id=WAYPOINT_START_ID; getline(fin,s); id++){
        //use the string 's' as input stream, the usage of 'sin' is just like 'cin'
        std::istringstream sin(s);
        double x,y,rz,rw;
        sin>>x;
        sin>>y;
        sin>>rz;
        sin>>rw;
        Waypoint_Ptr wp(new WaypointWrapper(id, x, y, rz, rw));

        ROS_INFO_STREAM("Loaded waypoint["<<id<<"]: "<<x<<", "<<y<<", "<<rz<<", "<<rw);
        AddWaypoint(wp);
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

    m_waypointClients.clear();
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
        bool sentSuccessfully = false;
        while(!sentSuccessfully)
        {
            ros::spinOnce();

            if (m_waypointClients[robotID].isValid())
            {
                ROS_INFO_STREAM("sending waypoint to robot: "<<robotID);
                if (m_waypointClients[robotID].call(s))
                {
                    if (s.response.result == 0)
                    {
                        sentSuccessfully = true;
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
                }
            }
            else
            {
                ROS_ERROR_STREAM("waypoint send service not setup properly. robot = "<<robotID);

                bool success = ros::service::waitForService(Conversion::RobotIDToWaypointTopic(robotID), ros::Duration(1.0));
                if (success)
                {
                    m_waypointClients[robotID] = m_nh->serviceClient<global_planner::WaypointFinished>(Conversion::RobotIDToWaypointTopic(robotID), true);
                }
                else
                {
                    ROS_ERROR_STREAM_THROTTLE(1.0, "Waiting on the waypoint service to become available");
                }
            }
        }
    }
    else
    {
        ROS_ERROR_STREAM("Sending waypoint. invalid id's: wp = "<<wpID<<", robot = "<<robotID);
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
int TaskMaster::SendDump(int dumpID)
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


        // TODO: if call1 fails, cancel the rest of the dump
        //  if call1 succeeds, but call 2 fails, cancel call1's dump through the
        //  calling function. make this function return a 0 if succeed, and 1 or 2
        //  if it fails.
        //
        //  If the dump fails, reset the status of the task (unassigned, or whatever)
        //   and try to assign the robots the next time through the loop
        //

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

                return 1;
            }
            else
            {
                ROS_INFO_STREAM("successfully sent dump("<<dm.id<<") to robot "<<dm.robotID1);
            }
        }
        else
        {
            ROS_ERROR_STREAM("Service call 1 failed for dump id: " << dm.id <<
                             ", Robot1(" << dm.robotID1 << "), Robot2(" <<
                             dm.robotID2 << ")");

            return 1;
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
                return 2;
            }
            else
            {
                ROS_INFO_STREAM("successfully sent dump("<<dm.id<<") to robot "<<dm.robotID2);
            }
        }
        else
        {
            ROS_ERROR_STREAM("Service call 2 failed for dump id: " << dm.id <<
                             ", Robot1(" << dm.robotID1 << "), Robot2(" <<
                             dm.robotID2 << ")");
            return 2;
        }

        // All passed
        return call1 && call2 && response1 && response2;
    }
    else {
        ROS_ERROR_STREAM("Invalid dump or robot ids: " << dm.id << ", Robot1(" << dm.robotID1 << "), Robot2(" << dm.robotID2 << ")");
        return -1;
    }
    return 0;
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
 *  Method: TaskMaster::GetWaypoints
 *  Params:
 * Returns: std::vector<Waypoint_Ptr>
 * Effects: Get all waypoints
 ***********************************************************************/
std::map<int, Waypoint_Ptr> TaskMaster::GetGoals()
{
    std::map<int, Waypoint_Ptr> m_goalMap;
    for (std::map<int, Waypoint_Ptr>::iterator it = m_waypointMap.begin(); it != m_waypointMap.end(); ++it)
    {
        // If it's avilable for task setting
        if (IsGoal(it->first))
        {
            m_goalMap[it->first] = m_waypointMap[it->first];
        }
    }
    return m_goalMap;
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
 /*
bool TaskMaster::cb_goalFinished(global_planner::GoalFinished::Request  &req,
                                 global_planner::GoalFinished::Response &res)
{
    int status = req.status;
    m_goalMap[req.id]->SetStatus(Conversion::IntToTaskResult(status));
    if (status == TaskResult::SUCCESS)
    {
        SendSound("mario_coin.wav");
        std::stringstream ss;
        ss<<"SUCCESS: Goal ("<<req.id<<") finished successfully";
        SendText(ss.str());
        ROS_INFO_STREAM("Goal finished successfully");
    }
    else
    {
        ROS_ERROR_STREAM("ERROR, Goal finished with status: " << req.status
                         << " : " << Conversion::TaskResultToString(Conversion::IntToTaskResult(req.status)));
        std::stringstream ss;
        ss << "ERROR: Goal ("<<req.id<<") not finished successfully " << Conversion::TaskResultToString(Conversion::IntToTaskResult(req.status));
        SendText(ss.str());
    }
}
*/


/***********************************************************************
 *  Method: TaskMaster::cb_waypointFinished
 *  Params: const global_planner::WaypointFinished::ConstPtr& msg
 * Returns: void
 * Effects: callback for when a waypoint is finished
 ***********************************************************************/
bool TaskMaster::cb_waypointFinished(global_planner::WaypointFinished::Request  &req,
                                     global_planner::WaypointFinished::Response &res)
{
    int status = req.status;
    m_waypointMap[req.id]->SetStatus(Conversion::IntToTaskResult(status));

    if (status == TaskResult::SUCCESS)
    {
        if (IsWaypoint(req.id))
        {
            ROS_INFO_STREAM("Waypoint["<<req.id<<"] reached successfully");
            SendSound("waypoint_reached.wav");

            std::stringstream ss;
            ss << "SUCCESS: Waypoint ("<<req.id<<") finished successfully";
            SendText(ss.str());
        }
        else
        {
            SendSound("goal_reached.wav");
            std::stringstream ss;
            ss<<"SUCCESS: Goal ("<<req.id<<") finished successfully";
            SendText(ss.str());
            ROS_INFO_STREAM("Goal finished successfully");
        }
    }
    else
    {
        ROS_ERROR_STREAM("ERROR, Waypoint finished with status: " << req.status
                         << " : " << Conversion::TaskResultToString(Conversion::IntToTaskResult(req.status)));
        //FOR NOW, let's just say it's available after a failure
        m_waypointMap[req.id]->SetStatus(TaskResult::AVAILABLE);
        SendSound("navigation_aborted.wav");
        std::stringstream ss;
        ss << "ERROR: Waypoint ("<<req.id<<") not reached successfully";
        SendText(ss.str());
    }
    return true;
}


/***********************************************************************
 *  Method: TaskMaster::cb_dumpFinished
 *  Params: const global_planner::DumpFinished::ConstPtr& msg
 * Returns: void
 * Effects: callback for when a dump is finished
 ***********************************************************************/
bool TaskMaster::cb_dumpFinished(global_planner::DumpFinished::Request  &req,
                                 global_planner::DumpFinished::Response &res)
{
    int status = req.status;
    int robotID = req.robotID; // TODO: Track which robot has finished dump
    if (status == TaskResult::SUCCESS)
    {
        if (m_dumpMap[req.id]->GetStatus() == TaskResult::INPROGRESS)
        {
            m_dumpMap[req.id]->SetStatus(TaskResult::DUMP_HALF_DONE);
            ROS_INFO_STREAM("Dumping halfway done");
            std::stringstream ss;
            ss << "SUCCESS: Dump ("<<req.id<<") half way done";
            SendText(ss.str());
        }
        else if (m_dumpMap[req.id]->GetStatus() == TaskResult::DUMP_HALF_DONE)
        {
            m_dumpMap[req.id]->SetStatus(TaskResult::DUMP_FINISHED);
            ROS_INFO_STREAM("Dumping was successful");
            SendSound("trash_transferred.wav");
            std::stringstream ss;
            ss << "ERROR: Dump ("<<req.id<<") reached by both robots successfully";
            SendText(ss.str());
        }
        else
        {
            ROS_INFO_STREAM("Ignoring received dumpFinished message due to being in state: " << m_dumpMap[req.id]->GetStatus() );
        }
    }
    else
    {
        ROS_ERROR_STREAM("Dump failed due to status: " << req.status<<" : "<<Conversion::TaskResultToString(Conversion::IntToTaskResult(req.status)));
            std::stringstream ss;
            ss << "Dump failed due to status: " << req.status<<" : "<<Conversion::TaskResultToString(Conversion::IntToTaskResult(req.status));
            SendText(ss.str());
            SendSound("failed_trash_transfer.wav");
            m_dumpMap[req.id]->SetStatus(TaskResult::FAILURE);
    }
    return true;
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

    std::map<int, Waypoint_Ptr>::iterator it = m_waypointMap.find(goalID);

    // it is already in the map... update?
    if(it != m_waypointMap.end())
    {
        Waypoint_Ptr currentGoal = m_waypointMap[goalID];
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

                //TODO: resend goal
                // SendGoal(goalID);
            }
        }
    }
    else //Not yet in the map
    {
        ros::Time time = msg->time;
        geometry_msgs::Pose pose = msg->pose;

        Waypoint_Ptr ptr(new WaypointWrapper());
        ptr->SetPose(pose);
        ptr->SetID(goalID);
        ptr->SetTime(time);
        ptr->SetStatus(TaskResult::AVAILABLE);
        ptr->SetRobot(-1);
        ROS_INFO_STREAM("Creating new goal in map: "<<goalID<<" : "<<ptr->ToString());
        ROS_INFO_STREAM("Adding to map");
        m_waypointMap[goalID] = ptr;
    }
}


/***********************************************************************
 *  Method: TaskMaster::GetAvailableGoals
 *  Params:
 * Returns: std::vector<Waypoint_Ptr>
 * Effects: Get all available goals
 ***********************************************************************/
std::vector<Waypoint_Ptr> TaskMaster::GetAvailableGoals()
{
    std::vector<Waypoint_Ptr> v;
    std::map<int, Waypoint_Ptr> m_goalMap = GetGoals();

    for (std::map<int, Waypoint_Ptr>::iterator it = m_goalMap.begin(); it != m_goalMap.end(); ++it)
    {
        // If it's avilable for task setting
        if (IsAvailable(it->first))
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
        if (IsAvailable(it->first))
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
        if (IsAvailable(it->first) || IsInProgress(it->first))
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


/***********************************************************************
 *  Method: TaskMaster::SendSound
 *  Params: std::string filename
 * Returns: void
 * Effects: play the sound specified in the filename
 *  (relaative to the 'resources/sounds' folder)
 ***********************************************************************/
void TaskMaster::SendSound(std::string filename)
{
    std_msgs::String s;
    s.data = filename;
    m_soundPub.publish(s);
}


/***********************************************************************
 *  Method: TaskMaster::SendSound
 *  Params: std::string filename
 * Returns: void
 * Effects: send text to the human interface node
 ***********************************************************************/
void TaskMaster::SendText(std::string text)
{
    std_msgs::String s;
    s.data = text;
    m_textPub.publish(s);
}


bool TaskMaster::IsWaypoint(int taskID)
{
    if (taskID >= WAYPOINT_START_ID)
    {
        return true;
    }
    return false;
}


bool TaskMaster::IsGoal(int taskID)
{
    if (IsWaypoint(taskID) == false)
    {
        return true;
    }
    return false;
}


bool TaskMaster::IsAvailable(int taskID)
{
    if (m_waypointMap[taskID]->GetStatus() == TaskResult::AVAILABLE)
        // m_waypointMap[taskID]->GetStatus() == TaskResult::INPROGRESS)
        return true;
    return false;
}

bool TaskMaster::IsInProgress(int taskID)
{
    if (m_waypointMap[taskID]->GetStatus() == TaskResult::INPROGRESS)
        return true;
    return false;
}