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

    ros::Rate r(50);

    while(ros::ok() && running == false)
    {
        ROS_INFO_THROTTLE(10, "Waiting on program start");
        ros::spinOnce();
        r.sleep();
    }

    gp.SendSound("mario_1_up.wav");
    ros::spinOnce();

    ros::Time start_time = ros::Time::now();
    ROS_INFO("Global Planner Started");
    while(ros::ok() && running == true && !gp.isFinished())
    {
        gp.Execute();
        r.sleep();
    }
    // gp.Finished(); // causes system exit
    ROS_INFO("Global Planner finished in : %g seconds", (ros::Time::now() - start_time).toSec() );
    gp.SendSound("mario_1_up.wav");

    ROS_INFO("GP Finished");
}
