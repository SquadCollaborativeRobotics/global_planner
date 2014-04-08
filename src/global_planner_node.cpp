#include <ros/ros.h>
#include <global_planner/GlobalPlanner.h>


bool running = false;

void toggle_planner(const std_msgs::Empty::ConstPtr& msg)
{
    running = !running;
}

int main(int argc, char** argv){
    // ROS Node Initialization
    ros::init(argc, argv, "global_planner_node");
    ros::NodeHandle nh;

    ROS_INFO("Initializing Global Planner");
    GlobalPlanner gp;

    gp.Init(&nh);
    sleep(1);
    gp.SendSound("beep.wav");

    ros::Subscriber toggleSub = nh.subscribe("/toggle_planner", 10, toggle_planner);

    ros::Rate r(15);

    while(ros::ok() && running == false)
    {
        ROS_INFO_THROTTLE(10, "Waiting on program start");
        ros::spinOnce();
        r.sleep();
    }

    ros::spinOnce();

    ROS_INFO("Node Starting Global Planner...");
    gp.Start();

    while(ros::ok() && running == true && !gp.isFinished())
    {
        gp.Execute();
        r.sleep();
    }
    gp.Finished();

    ROS_INFO("GP Node Finished");
}
