#include <ros/ros.h>
#include <global_planner/RobotController.h>

int main(int argc, char** argv){
    // ROS Node Initialization
    ros::init(argc, argv, "robot_controller_node");
    ros::NodeHandle nh;

    ROS_ERROR("Robot Controller Started");
    RobotController rc;

    // Nodehandle, Robot id, name, capacity, used, type
    rc.Init(&nh, 0, "collector1", 3, 0, true);

    ros::Rate r(40);

    while(ros::ok())
    {
        rc.Execute();
        r.sleep();
    }

    rc.Finished();

    ROS_INFO("Finished");
}
