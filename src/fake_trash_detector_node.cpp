#include <ros/ros.h>
#include <global_planner/RobotStatusWrapper.h>
#include <geometry_msgs/Pose.h>

/*
Has a list of trash can pick-up position poses which are 'hidden'
Tracks all robot x/y positions
When a robot is able to 'see' a trash can, it adds the waypoint to the taskmaster list of waypoints.
That is all
*/

std::map<int, geometry_msgs::Pose> robot_poses;

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
        ROS_INFO_STREAM(it->first << " : x:" << it->second.position.x << ", y:" << it->second.position.x);
    }
}

int main(int argc, char** argv)
{
    // ROS Node Initialization
    ros::init(argc, argv, "fake_trash_detector");
    ros::NodeHandle nh;
    ROS_INFO("Initializing Fake Trash Detector...");

    // Track robots
    ros::Subscriber robotStatusSub = nh.subscribe("/robot_status", 10, updateRobotPose);

    ros::Time last_time = ros::Time::now();

    ros::Rate loop_rate(5); // 5 Hz

    ROS_INFO("Initialized. Starting Fake Trash Detector Loop.");
    while (ros::ok())
    {    
        // Print out map every N seconds
        if ((ros::Time::now() - last_time) > ros::Duration(5))
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