#include <ros/ros.h>
#include <ros/package.h>
#include <global_planner/RobotStatusWrapper.h>
#include <global_planner/WaypointWrapper.h>
#include <geometry_msgs/Pose.h>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

/*
Has a list of trash can pick-up position poses which are 'hidden'
Tracks all robot x/y positions
When a robot is able to 'see' a trash can, it adds the waypoint to the taskmaster list of waypoints.
That is all
*/

#define WAYPOINT_START_ID 1 // Start id for waypoints read from file
#define DISTANCE_THRESHOLD 0.5 // meters, threshold distance where trashcan is 'seen' by a robot (50cm = 0.5)

std::map<int, geometry_msgs::Pose> robot_poses;
// List of all trashcan waypoints (all of them loaded at start, visible or not)
// The waypoints are the pose directly in front of the trash can (the place we'd want the robot to go)
std::map<int, Waypoint_Ptr> trashcan_waypoints; 
// Tracks which trashcans have already been published (so we only do them once)
std::set<int> published_trashcans;

// Trashcan waypoint publisher, publishes new trashcan when visible only once per trashcan
ros::Publisher trashPub;

// Callback to update pose of robot in robot_poses map
void updateRobotPose(const global_planner::RobotStatus::ConstPtr& msg)
{
    robot_poses[msg->id] = msg->pose; // Create or update entry in map
}

// pretty prints out robot poses to ros info
void printRobotPoses()
{
    ROS_INFO("Robot Poses:");
    for (std::map<int, geometry_msgs::Pose>::iterator it = robot_poses.begin(); it != robot_poses.end(); ++it)
    {
        // ROS_INFO_STREAM(it->first << " : x:" << it->second);
        ROS_INFO_STREAM(it->first << " : x:" << it->second.position.x << ", y:" << it->second.position.y);
    }
    ROS_INFO("Trashcan Poses:");
    for (std::map<int, Waypoint_Ptr>::iterator it = trashcan_waypoints.begin(); it != trashcan_waypoints.end(); ++it)
    {
        ROS_INFO_STREAM(it->first << " : " << it->second->ToString());
    }
}

double get2DPoseDistance(geometry_msgs::Pose a, geometry_msgs::Pose b) {
    double dx = (b.position.x - a.position.x);
    double dy = (b.position.y - a.position.y);
    return sqrt( dx*dx + dy*dy );
}

void publishTrash(int trash_id)
{
    trashPub.publish( trashcan_waypoints[trash_id]->GetMessage() );
    published_trashcans.insert(trash_id); // Add to published trashcan set so we don't republish
}

bool canRobotSeeTrash(int robot_id, int trash_id)
{
    // If it's visible to the robot
    if (get2DPoseDistance(robot_poses[robot_id], trashcan_waypoints[trash_id]->GetPose()) < DISTANCE_THRESHOLD)
    {
        // Trashcan visible, publish it (if it hasn't yet)
        ROS_INFO_STREAM("SEEN : Robot " << robot_id << " 'sees' Trash Waypoint" << trash_id);
        publishTrash(trash_id);
        return true;
    }
    return false;
}

bool hasPublishedTrash(int trash_id)
{
    return published_trashcans.find(trash_id) != published_trashcans.end();
}

// Check all trashcans against all robots for visibility
void doTrashVisibilityCheck()
{
    for (std::map<int, Waypoint_Ptr>::iterator trash_it = trashcan_waypoints.begin(); trash_it != trashcan_waypoints.end(); ++trash_it)
    {
        // Only check trashcan if it hasn't been seen yet
        if (!hasPublishedTrash(trash_it->first))
        {
            // For each robot pose, break on first success
            for (std::map<int, geometry_msgs::Pose>::iterator robot_it = robot_poses.begin(); robot_it != robot_poses.end(); ++robot_it)
            {
                if (canRobotSeeTrash(robot_it->first, trash_it->first))
                    break;
            }
        }
    }   
}


void loadWaypointsFromFile(std::string filename)
{
    ROS_INFO_STREAM("Loading waypoints from file: " << filename);
    std::string filepath = ros::package::getPath("global_planner");
    filepath += std::string("/resources/waypoint_lists/");
    filepath += filename;
    std::ifstream fin(filepath.c_str());
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

        ROS_INFO_STREAM("Loaded waypoint[" << id << "]: " << x << ", " << y << ", " << rz << ", " << rw);
        trashcan_waypoints[id] = wp;
    }
    fin.close();
}

int main(int argc, char** argv)
{
    // ROS Node Initialization
    ros::init(argc, argv, "fake_trash_detector");
    ros::NodeHandle nh;
    ROS_INFO("Initializing Fake Trash Detector...");

    std::string waypointFile("testList1.points");
    nh.getParam("trashcans_file", waypointFile); // Update waypointFile if it exists
    ROS_INFO_STREAM("Loading fake trash waypoints from: "<<waypointFile);
    loadWaypointsFromFile(waypointFile);

    // Publisher for trash waypoints
    trashPub = nh.advertise<global_planner::WaypointMsg>("/fake_trashcans", 100);

    // Add fake robot
    geometry_msgs::Pose p = Conversion::SetPose(2.9,0,0,0);
    robot_poses[13] = p;

    // Track robots
    ros::Subscriber robotStatusSub = nh.subscribe("/robot_status", 10, updateRobotPose);

    ros::Time last_time = ros::Time::now();

    ros::Rate loop_rate(1); // 1 Hz

    ROS_INFO("Initialized. Starting Fake Trash Detector Loop.");
    while (ros::ok())
    {   
        // Check if a robot is near a trashcan
        // If yes, and the trashcan hasn't been published yet, publish it.
        doTrashVisibilityCheck();

        // Print out map every N seconds
        if ((ros::Time::now() - last_time) > ros::Duration(2))
        {
            printRobotPoses();
            last_time = ros::Time::now();
        }


        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Fake Trash Detector Loop Finished.");

    return 0;
}