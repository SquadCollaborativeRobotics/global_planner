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

    ROS_ERROR("Global Planner Started");
    GlobalPlanner gp;

    gp.Init(&nh);
    sleep(1);
    gp.SendSound("beep.wav");

    ros::Subscriber toggleSub = nh.subscribe("/toggle_planner", 10, toggle_planner);

    ros::Rate r(50);

    while(ros::ok() && running == false)
    {
        ROS_INFO_THROTTLE(3, "Waiting on program start");
        ros::spinOnce();
        r.sleep();
    }

    gp.SendSound("mario_1_up.wav");
    ros::spinOnce();

    while(ros::ok() && running == true)
    {
        gp.Execute();
        r.sleep();
    }

    gp.Finished();

    ROS_INFO("GP Finished");
}
