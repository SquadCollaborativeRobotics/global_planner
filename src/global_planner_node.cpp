#include "ros/ros.h"
#include "global_planner/GlobalPlanner.h"

int main(int argc, char** argv){
    // ROS Node Initialization
    ros::init(argc, argv, "global_planner_node");
    ros::NodeHandle nh;

    ROS_INFO("Global Planner Started");
    GlobalPlanner gp;

    gp.Init(nh);

    while(ros::ok())
    {
        gp.Execute();
    }

    gp.Finished();

    ROS_INFO("Finished");
}
