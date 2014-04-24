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


    ros::spinOnce();

    ROS_INFO("Node Starting Global Planner...");
    gp.Start();

    ros::Time lastDisplay = ros::Time::now();

    ros::Rate r(1);
    while(ros::ok() && !gp.isFinished())
    {
        if (running)
        {
            gp.Execute();
        }
        else
        {
            gp.QueryRobots();
            ROS_INFO_THROTTLE(20, "Global Planner is paused");
            if ((ros::Time::now() - lastDisplay) > ros::Duration(15))
            {
                gp.Display();
                lastDisplay = ros::Time::now();
            }
        }
        r.sleep();
        ros::spinOnce();
    }
    gp.Finished();

    ROS_INFO("GP Node Finished. Press any key to exit");
    std::string end;
    std::cin>>end;
}
