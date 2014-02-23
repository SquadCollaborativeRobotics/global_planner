#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread.hpp>
#include "global_planner/SoundMsg.h"
#include "string.h"

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
    void sound_thread(std::string str, int numTimes)
    {
        try
        {
            std::string path_to_sound = std::string("aplay -q ");
            path_to_sound += ros::package::getPath("global_planner");
            path_to_sound += std::string("/resources/sounds/");
            path_to_sound += str;

            //Play the sound for the number of times specified
            for (int i = 0; i < numTimes; ++i)
            {
                if (m_stopThread)
                {
                    system("killall -9 aplay");
                    break;
                }
                else
                {
                    if (i > 1){
                        sleep(1.0);
                    }
                    ROS_INFO_STREAM("Playing sound: "<<path_to_sound);
                    is_playing = true;
                    system(path_to_sound.c_str());
                    is_playing = false;
                }
            }
        }
        catch(boost::thread_interrupted&)
        {
            system("killall -9 aplay");
            return;
        }
    };

    // Thread function
    void play_sound(std::string str, int numTimes)
    {
        //thread has not yet finished. try killing.
        m_stopThread = true;
        system("killall -9 aplay");
        m_soundThread.join();
        m_stopThread = false;
        m_soundThread = boost::thread(&SoundManager::sound_thread, this, str, numTimes);
    };
    bool m_stopThread;
    boost::thread m_soundThread;
};

SoundManager sound_manager;

void sound_callback(const global_planner::SoundMsg::ConstPtr& msg) {
    std::string str = msg->filename;
    int numTimes = msg->num_times;
    std::string text_output = msg->text_output;
    if (text_output.length() > 0)
        ROS_INFO_STREAM(text_output);
    sound_manager.play_sound(str, numTimes);
}

int main(int argc, char** argv){
    // ROS Node Initialization
    ros::init(argc, argv, "human_interaction_node");
    ros::NodeHandle nh;

    // Subscribe to command node
    ros::Subscriber sound_sub = nh.subscribe("play_sound", 1, sound_callback);

    ROS_INFO("Starting interaction node");

    while(ros::ok())
    {
        ros::spinOnce();
    }
}
