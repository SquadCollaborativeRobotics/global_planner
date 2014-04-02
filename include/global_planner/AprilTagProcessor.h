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

#include <boost/shared_ptr.hpp>

#define STOP_LIN_THRESHOLD 0.01
#define STOP_ANG_THRESHOLD 0.01

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

private:
    void cb_aprilTags(const april_tags::AprilTagList::ConstPtr &msg);


    bool UpdatePose();

    ros::Time LastSeenTime(int tagID);
    void GetLandmarks(std::vector<int> ret);
    void GetGoals(std::vector<int> ret);


    AprilTagProcessor::TAG_TYPE GetType(int tagID);
    bool IsLandmarkVisible();
    double GetDistance(tf::StampedTransform &tf);
    bool PosesDiffer(geometry_msgs::PoseWithCovariance& poseWithCovariance1, geometry_msgs::PoseWithCovariance& poseWithCovariance2);
    void PrintTransform(tf::StampedTransform& transform);

    bool GetMapTransform(std::string frame_name, tf::StampedTransform &tf);
    bool GetTransform(std::string frame_name1, std::string frame_name2, tf::StampedTransform &tf);

    std::string GetTagFrameName(int tagID);
    std::string GetLandmarkFrameName(int tagID);
    int GetIDFromFrameName(std::string frame_name);

    // A map that keeps track of all tags seen and the time it was last seen (using the pose stamped time)
    std::map<int, geometry_msgs::PoseStamped> m_pose;
    std::map<int, AprilTagProcessor::TAG_TYPE> m_goalTypeMap;
    ros::Time m_lastImageTime;
    ros::Time m_lastLocalizeTime;

    boost::shared_ptr<tf::TransformListener> m_tfListener;
    int m_robotID;
    bool m_shouldPause;

    // AMCL pose
    ros::Subscriber m_amclPoseSub;
    geometry_msgs::PoseWithCovarianceStamped m_amclPose;

    // current odometry of the robot. mainly used for velocity
    ros::Subscriber m_odomSub;
    nav_msgs::Odometry m_odom;
    bool IsStopped();

    //Subscribe to April Tag messages from the robot
    ros::Subscriber m_tagSub;
};
