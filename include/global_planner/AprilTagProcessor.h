/**
 * The April Tags Processor class manages the april tag messages coming in from cameras and
 *  determining what to do based on the incoming message
 */

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <stdlib.h>
#include <sstream>
#include <april_tags/AprilTagList.h>
#include <april_tags/AprilTag.h>
#include <tf/transform_listener.h>
#include <global_planner/GoalSeen.h>
#include <std_msgs/String.h>

#include <boost/shared_ptr.hpp>

#define STOP_LIN_THRESHOLD 0.01 //Meters/sec
#define STOP_ANG_THRESHOLD 0.01 // Radians/sec
#define UPDATE_RANGE_THRESHOLD 1.5 //meters

class AprilTagProcessor
{
public:
    AprilTagProcessor();
    ~AprilTagProcessor();

    enum TAG_TYPE
    {
        LANDMARK = 0,
        GOAL = 1,
        UNKNOWN = 10
    };

    bool Init(ros::NodeHandle *nh, int robotID);
    bool Execute();

    /**
     * These functions will be used by the robot controller to transition between states
     */
    bool ShouldPause();
    bool ShouldResume();
    bool SetShouldPause(bool shouldPause);

private:
    void cb_aprilTags(const april_tags::AprilTagList::ConstPtr &msg);
    void cb_goalSeen(const global_planner::GoalSeen::ConstPtr &msg);
    void cb_odom(const nav_msgs::Odometry::ConstPtr& msg);
    void cb_amclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    bool UpdatePose();
    bool FindGoals();

    ros::Time LastSeenTime(int tagID);
    void UpdateLastSeenTime(int tagID, ros::Time time);
    ros::Time LastGoalSendTime(int tagID);
    void GetLandmarks(std::vector<int>& ret);
    void GetGoals(std::vector<int>& ret);


    AprilTagProcessor::TAG_TYPE GetType(int tagID);
    bool IsLandmarkVisible();
    bool IsStopped();

    double GetDistance(tf::StampedTransform &tf);
    double GetDistance(geometry_msgs::PoseStamped &pose);
    bool PosesDiffer(geometry_msgs::PoseWithCovariance& poseWithCovariance1, geometry_msgs::PoseWithCovariance& poseWithCovariance2);
    void PrintTransform(tf::StampedTransform& transform);

    bool GetMapTransform(std::string frame_name, tf::StampedTransform &tf, ros::Time=ros::Time(0));
    bool GetTransform(std::string frame_name1, std::string frame_name2, tf::StampedTransform &tf, ros::Time=ros::Time(0));

    std::string GetTagFrameName(int tagID);
    std::string GetLandmarkFrameName(int tagID);
    int GetIDFromFrameName(std::string frame_name);

    /***************************************************************
     * MEMBER VARIABLES
     **************************************************************/

    std::string m_cameraFrame;
    std::string m_robotBaseFrame;
    ros::NodeHandle *m_nh;

    // A map that keeps track of all tags seen and the time it was last seen (using the pose stamped time)
    std::map<int, geometry_msgs::PoseStamped> m_pose;

    // Map that keeps track of the tag's type
    std::map<int, AprilTagProcessor::TAG_TYPE> m_goalTypeMap;

    // Map that keeps track of when the last time this goal was published
    std::map<int, ros::Time> m_goalSendTime;
    std::map<int, ros::Time> m_timeSeenClose;
    ros::Time m_lastImageTime;
    ros::Time m_lastLocalizeTime;

    boost::shared_ptr<tf::TransformListener> m_tfListener;
    int m_robotID;
    bool m_shouldPause;

    bool m_seesGoal;
    bool m_seesLandmark;

    //goals already seen in the environment
    std::map<int, ros::Time> m_goalsRegistered;
    ros::Subscriber m_goalSeenSub;
    bool GoalRegistered(int id);

    /***************************************************************
     * AMCL pose
     **************************************************************/
    //Subscribes to the amcl pose message
    ros::Subscriber m_amclPoseSub;
    geometry_msgs::PoseWithCovarianceStamped m_amclPose;
    // Publisher that publishes a pose for visualization purposes to the "/new_pose"
    ros::Publisher m_newPosePub;
    // Publisher that reinitializes the pose of the robot (relocalizes)
    ros::Publisher m_newInitialPosePub;

    // Publisher that sends out an april tag that is a possible goal node?
    ros::Publisher m_goalPub;

    // current odometry of the robot. mainly used for velocity
    ros::Subscriber m_odomSub;
    nav_msgs::Odometry m_odom;

    //Subscribe to April Tag messages from the robot
    ros::Subscriber m_tagSub;



    void SendSound(std::string filename);
    void SendText(std::string text);

    // Sound message publisher
    ros::Publisher m_soundPub;
    // Sound message publisher
    ros::Publisher m_textPub;
};
