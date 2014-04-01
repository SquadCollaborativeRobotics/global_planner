/***********************************************************************
 * AUTHOR: shawn <shawn>
 *   FILE: src/global_planner/src//AprilTagProcessor.cpp
 *   DATE: Thu Mar 20 13:19:00 2014
 *  DESCR:
 ***********************************************************************/
#include <global_planner/AprilTagProcessor.h>

/***********************************************************************
 *  Method: AprilTagProcessor::AprilTagProcessor
 *  Params:
 * Effects: Default Constructor
 ***********************************************************************/
AprilTagProcessor::AprilTagProcessor():
m_robotID(-1),
m_shouldPause(false)
{
}


/***********************************************************************
 *  Method: AprilTagProcessor::~AprilTagProcessor
 *  Params:
 * Effects:
 ***********************************************************************/
AprilTagProcessor::~AprilTagProcessor()
{
}


/***********************************************************************
 *  Method: AprilTagProcessor::Init
 *  Params: int robotID
 * Effects: Initialize with
 ***********************************************************************/
bool AprilTagProcessor::Init(ros::NodeHandle *nh, int robotID)
{
    m_tagSub = nh->subscribe("april_tags", 100, &AprilTagProcessor::cb_aprilTags, this);
    m_robotID = robotID;
    m_tfListener.reset( new tf::TransformListener(*nh) );
}


/***********************************************************************
 *  Method: AprilTagProcessor::Execute
 *  Params:
 * Effects:
 ***********************************************************************/
bool AprilTagProcessor::Execute()
{
}


/***********************************************************************
 *  Method: AprilTagProcessor::ShouldPause
 *  Params:
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool AprilTagProcessor::ShouldPause()
{
    return m_shouldPause;
}


/***********************************************************************
 *  Method: AprilTagProcessor::ShouldResume
 *  Params:
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool AprilTagProcessor::ShouldResume()
{
    return !m_shouldPause;
}


/***********************************************************************
 *  Method: AprilTagProcessor::UpdatePose
 *  Params: int tagID
 * Returns: bool
 * Effects: Update the pose using the tag specified in the parameter
 ***********************************************************************/
bool AprilTagProcessor::UpdatePose()
{
    bool retVal = false;

    return retVal;
}


/***********************************************************************
 *  Method: AprilTagProcessor::GetLandmarks
 *  Params: std::vector<int> vec
 * Returns: void
 * Effects:
 ***********************************************************************/
void AprilTagProcessor::GetLandmarks(std::vector<int> vec)
{
    vec.clear();
    for (std::map<int, AprilTagProcessor::TAG_TYPE>::iterator i = m_goalTypeMap.begin(); i != m_goalTypeMap.end(); ++i)
    {
        if (i->second == LANDMARK)
        {
            vec.push_back(i->first);
        }
    }
}

/***********************************************************************
 *  Method: AprilTagProcessor::GetGoals
 *  Params: std::vector<int> vec
 * Returns: void
 * Effects:
 ***********************************************************************/
void AprilTagProcessor::GetGoals(std::vector<int> vec)
{
    vec.clear();
    for (std::map<int, AprilTagProcessor::TAG_TYPE>::iterator i = m_goalTypeMap.begin(); i != m_goalTypeMap.end(); ++i)
    {
        if (i->second == GOAL)
        {
            vec.push_back(i->first);
        }
    }
}


/***********************************************************************
 *  Method: AprilTagProcessor::LastSeenTime
 *  Params: int tagID
 * Returns: ros::Time
 * Effects:
 ***********************************************************************/
ros::Time AprilTagProcessor::LastSeenTime(int tagID)
{
    return m_pose[tagID].header.stamp;
}


/***********************************************************************
 *  Method: AprilTagProcessor::IsStopped
 *  Params:
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool AprilTagProcessor::IsStopped()
{
    if (m_odom.twist.twist.linear.x > STOP_LIN_THRESHOLD ||
        m_odom.twist.twist.linear.y > STOP_LIN_THRESHOLD ||
        m_odom.twist.twist.angular.z > STOP_ANG_THRESHOLD)
        return false;
    return true;
}


/***********************************************************************
 *  Method: AprilTagProcessor::cb_aprilTags
 *  Params: const april_tags::AprilTagList::ConstPtr &msg
 * Returns: void
 * Effects:
 ***********************************************************************/
void AprilTagProcessor::cb_aprilTags(const april_tags::AprilTagList::ConstPtr &msg)
{
    int numTags = msg->tag_ids.size();

    if (numTags == 0)
    {
        ROS_INFO_THROTTLE(4.0, "No tags seen in the robot's camera this time");
    }
    else
    {
        ROS_INFO_STREAM(msg->poses.size()<<" tags seen in the robot's camera this time");
        for (int i = 0; i < numTags; ++i)
        {
            //m_timeSeen[msg->tag_ids[i]] = msg->poses[i].header.stamp;
        }
    }

    for (int i = 0; i < numTags; ++i)
    {
        if (m_pose[i].header.stamp != m_lastImageTime) {
            m_pose[msg->tag_ids[i]] = msg->poses[i];
            m_shouldPause = true;
        }
    }

    m_lastImageTime = msg->poses[0].header.stamp;

    Execute();
}
/***********************************************************************
 *  Method: AprilTagProcessor::GetType
 *  Params: int tagID
 * Returns: AprilTagProcessor::TAG_TYPE
 * Effects:
 ***********************************************************************/
AprilTagProcessor::TAG_TYPE AprilTagProcessor::GetType(int tagID)
{
}


/***********************************************************************
 *  Method: AprilTagProcessor::PosesDiffer
 *  Params: geometry_msgs::PoseWithCovariance &poseWithCovariance1, geometry_msgs::PoseWithCovariance &poseWithCovariance2
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool AprilTagProcessor::PosesDiffer(geometry_msgs::PoseWithCovariance &poseWithCovariance1, geometry_msgs::PoseWithCovariance &poseWithCovariance2)
{
}


/***********************************************************************
 *  Method: AprilTagProcessor::IsLandmarkVisible
 *  Params:
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool AprilTagProcessor::IsLandmarkVisible()
{
    //Get all valid landmarks
    std::vector<int> landmarks;
    GetLandmarks(landmarks);
    //Check through the landmarks to see if there are any currently in view
    for (int i = 0; i < landmarks.size(); ++i)
    {
        if (GetType(i) != LANDMARK)
        {
            // ROS_ERROR_STREAM("ERROR: upadting pose with tag ID ("<<i<<") failed. This tag is not a landmark");
            continue;
        }

        if (LastSeenTime(i) != m_lastImageTime)
        {
            // ROS_ERROR_STREAM("ERROR: Tag ID ("<<i<<") is not in view of the camera right now");
            continue;
        }

        return true;
    }
}
