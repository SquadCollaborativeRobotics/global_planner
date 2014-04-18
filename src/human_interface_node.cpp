#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread.hpp>
#include <std_msgs/String.h>
#include <string.h>

class SoundManager
{
public:
    SoundManager()
    {
        is_playing = false;
        m_stopThread = false;
    };
    ~SoundManager(){};

    bool is_playing;

    // ROS topic callback
    void sound_thread(std::string str)
    {
        try
        {
            std::string path_to_sound = std::string("aplay -q ");
            path_to_sound += ros::package::getPath("global_planner");
            path_to_sound += std::string("/resources/sounds/");
            path_to_sound += str;

            if (m_stopThread)
            {
                system("killall -q -9 aplay");
            }
            else
            {
                // ROS_INFO_STREAM("Playing sound: "<<path_to_sound);
                is_playing = true;
                system(path_to_sound.c_str());
                is_playing = false;
            }
        }
        catch(boost::thread_interrupted&)
        {
            system("killall -q -9 aplay");
            return;
        }
    };

    // Thread caller function
    void play_sound(std::string str)
    {
        //thread has not yet finished. try killing.
        m_stopThread = true;
        system("killall -q -9 aplay");
        m_soundThread.join();
        m_stopThread = false;
        m_soundThread = boost::thread(&SoundManager::sound_thread, this, str);
    };
    bool m_stopThread;
    boost::thread m_soundThread;
};

SoundManager sound_manager;

void sound_callback(const std_msgs::String::ConstPtr& msg) {
    std::string str = msg->data;
    sound_manager.play_sound(str);
}

void text_callback(const std_msgs::String::ConstPtr& msg) {
    std::string str = msg->data;
    ROS_INFO_STREAM(str);
}

int main(int argc, char** argv){
    // ROS Node Initialization
    ros::init(argc, argv, "human_interaction_node");
    ros::NodeHandle nh;

    // Subscribe to command node
    ros::Subscriber sound_sub = nh.subscribe("/interface_sound", 1, sound_callback);
    ros::Subscriber text_sub = nh.subscribe("/interface_text", 1, text_callback);

    ROS_INFO("Starting interaction node");

    ros::Rate r(10);

    ros::spin();
}
