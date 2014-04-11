#include <ros/ros.h>
#include <global_planner/RobotController.h>

int main(int argc, char** argv){
    // ROS Node Initialization
    ros::init(argc, argv, "robot_controller_node");
    ros::NodeHandle nh;

    ROS_INFO("Robot Controller Started");
    RobotController rc;

    rc.Init(&nh);

    ros::Rate r(40);

    while(ros::ok())
    {
        rc.Execute();
        r.sleep();
    }

    rc.Finished();

    ROS_INFO("Robot node Finished. Press any key to exit");

    std::string end;
    std::cin>>end;
}
