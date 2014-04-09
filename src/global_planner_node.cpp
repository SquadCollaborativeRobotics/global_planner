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

    ros::Rate r(5);

    // while(ros::ok())
    // {
    //     ROS_INFO_THROTTLE(10, "Waiting on program start");
    //     ros::spinOnce();
    //     r.sleep();
    // }

    ros::spinOnce();

    ROS_INFO("Node Starting Global Planner...");
    gp.Start();

    while(ros::ok() && !gp.isFinished())
    {
        if (running)
        {
            gp.Execute();
        }
        else
        {
            ROS_INFO_THROTTLE(5.0, "Global Planner is paused");
        }
        r.sleep();
        ros::spinOnce();
    }
    gp.Finished();

    ROS_INFO("GP Node Finished");
}
