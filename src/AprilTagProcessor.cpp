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
m_shouldPause(false),
m_seesGoal(false),
m_seesLandmark(false),
m_lastImageTime(ros::Time(0)),
m_lastLocalizeTime(ros::Time(0))
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

    m_goalTypeMap[3] = AprilTagProcessor::LANDMARK;
    m_goalTypeMap[5] = AprilTagProcessor::LANDMARK;
    m_goalTypeMap[8] = AprilTagProcessor::GOAL;
    m_goalTypeMap[6] = AprilTagProcessor::GOAL;
}


/***********************************************************************
 *  Method: AprilTagProcessor::Execute
 *  Params:
 * Effects:
 ***********************************************************************/
bool AprilTagProcessor::Execute()
{
    if (IsStopped())
    {
        FindGoals();
        UpdatePose();
        return true;
    }
    return false;
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
    //Wait at least a second before allowing the robot to resume
    if (ros::Time::now() - m_lastLocalizeTime > ros::Duration(1.0))
        return false;
    return !m_shouldPause;
}


/***********************************************************************
 *  Method: AprilTagProcessor::UpdatePose
 *  Params:
 * Returns: bool
 * Effects: Update the pose using the tag specified in the parameter
 ***********************************************************************/
bool AprilTagProcessor::UpdatePose()
{
    if (IsStopped() == false)
    {
        ROS_ERROR_STREAM_THROTTLE(0.5, "ERROR: Robot is not yet stopped");
        return false;
    }

    std::vector<int> landmarks;
    GetLandmarks(landmarks);
    int bestLandmarkId = -1;
    // double closestLandmark = 9999999;

    //Pick best tag to use for localization (assuming there are several options)
    for (int i = 0; i < landmarks.size(); ++i)
    {
        int tagID = landmarks[i];
        ros::Time tagSeenTime = LastSeenTime(tagID);
        if (tagSeenTime == m_lastImageTime)
        {
            // closestLandmark = dist;
            bestLandmarkId = tagID;
        }
    }

    if (bestLandmarkId < 0)
    {
        ROS_ERROR_STREAM("No landmarks are in camera frame");
        return false;
    }

    //Update the pose of the robot
    //
    //First, get the frame names: (one for landmark, one for april tag)

    std::string landmark_frame;
    std::string april_frame;

    tf::StampedTransform mapLandmarkTF;
    if (GetMapTransform(landmark_frame, mapLandmarkTF))
    {
        tf::StampedTransform tagCameraTF;

        std::string tagFrameString = GetTagFrameName(bestLandmarkId);
        if (GetTransform(tagFrameString, m_cameraFrame, tagCameraTF))
        {
            tf::StampedTransform cameraBaseTF;
            if (GetTransform(m_cameraFrame, m_robotBaseFrame, cameraBaseTF))
            {
                // New amcl pose, in tf form
                tf::Transform newAMCLPoseTF = mapLandmarkTF * tagCameraTF * cameraBaseTF;

                // Convert to a geometry message, which is what AMCL accepts
                // New AMCL pose, in message form
                geometry_msgs::Transform newAMCLTFMsg;
                tf::transformTFToMsg(newAMCLPoseTF, newAMCLTFMsg);

                //AMCL takes the pose as a PoseWithCovarianceStamped. Build the poseWithCovariance first
                geometry_msgs::PoseWithCovariance poseWithCovariance;
                poseWithCovariance.pose.position.x = newAMCLTFMsg.translation.x;
                poseWithCovariance.pose.position.y = newAMCLTFMsg.translation.y;
                poseWithCovariance.pose.position.z = newAMCLTFMsg.translation.z;
                poseWithCovariance.pose.orientation = newAMCLTFMsg.rotation;

                // create the covariance array (the values are based on what rviz sends)
                // TODO: change based on confidence (distance to tag, speed of turning, etc)
                // row order covariance.... first row (0-5) = covariance from x, (6-12) = cov. from y, etc.
                // in this order: (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
                boost::array<float, 36> covariance = {
                    0.03, 0.0, 0.0, 0.0, 0.0, 0.0, // there is some variance in x due to moving in x
                    0.0, 0.03,0.0, 0.0, 0.0, 0.0,  // there is some variance in y due to moving in y
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // No z motion occurs... no variance
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // no x rotation occurs
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // no y rotation ccurs
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.05  // z rotation is fairly uncertain???
                };
                poseWithCovariance.covariance = covariance;

                // Create a "stamped" message
                geometry_msgs::PoseWithCovarianceStamped newRobotPoseMsg;
                newRobotPoseMsg.header.frame_id = "map";
                newRobotPoseMsg.header.stamp = tagCameraTF.stamp_;

                newRobotPoseMsg.pose = poseWithCovariance;

                //Use for display purposes...
                geometry_msgs::PoseStamped poseStamped;
                poseStamped.header.frame_id = "map";
                poseStamped.header.stamp = tagCameraTF.stamp_;
                poseStamped.pose = poseWithCovariance.pose;


                if (PosesDiffer(m_amclPose.pose, newRobotPoseMsg.pose))
                {
                    m_newPosePub.publish(poseStamped);
                    m_newInitialPosePub.publish(newRobotPoseMsg);
                    m_lastLocalizeTime = ros::Time::now();
                    m_shouldPause = false;

                    //wait for amcl to reinitialize
                    ROS_INFO_STREAM("Wait for AMCL to reinitialize");
                }
                else
                {
                    ROS_WARN_STREAM("Poses do not differ enough to need april tag localization");
                    m_shouldPause = false;
                }
                return true;
            }
            else
            {
                ROS_ERROR("Cannot transform from camera to robot base");
                return false;
            }
        }
        else
        {
            ROS_ERROR("Cannot transform from tag to camera");
            return false;
        }
    }
    else
    {
        ROS_ERROR("Cannot transform from map to landmark");
        return false;
    }

    return false;
}


/***********************************************************************
 *  Method: AprilTagProcessor::FindGoals
 *  Params:
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool AprilTagProcessor::FindGoals()
{
    if (IsStopped() == false)
    {
        ROS_ERROR_STREAM_THROTTLE(0.5, "ERROR: Robot is not yet stopped");
        return false;
    }

    std::vector<int> goals;
    GetGoals(goals);
    // double closestLandmark = 9999999;

    //Pick best tag to use for localization (assuming there are several options)
    for (int i = 0; i < goals.size(); ++i)
    {
        int tagID = goals[i];
        ros::Time tagSeenTime = LastSeenTime(tagID);

        if (tagSeenTime == m_lastImageTime)
        {
            std::string goal_frame = GetTagFrameName(tagID);

            tf::StampedTransform transform;
            if (GetTransform("map", goal_frame, transform))
            {
                global_planner::GoalMsg can;
                geometry_msgs::PoseStamped ps;
                tf::Quaternion quat;

                quat = transform.getRotation();

                // Get pose in map frame
                ps.header.frame_id = "map";

                //Set garbage pose based on the transform
                ps.pose.position.x = transform.getOrigin().x();
                ps.pose.position.y = transform.getOrigin().y();
                ps.pose.position.z = transform.getOrigin().z();

                //Set garbage rotation from the transform
                geometry_msgs::Quaternion quatMsg;
                quatMsg.x=quat.x();
                quatMsg.y=quat.y();
                quatMsg.z=quat.z();
                quatMsg.w=quat.w();
                ps.pose.orientation = quatMsg;

                //Finish creating message & publish
                can.pose = ps.pose;
                //TODO: make more correct
                can.id = 6;

                m_goalPub.publish(can);
                m_goalSendTime[tagID] = ros::Time::now();

                ROS_ERROR("Sent goal pose");
            }
        }
    }

    //Update the pose of the robot
    //
    //First, get the frame names: (one for landmark, one for april tag)

    return true;
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
    if (m_pose.find(tagID) != m_pose.end())
    {
        return m_pose[tagID].header.stamp;
    }
    return ros::Time(0);
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
    }

    for (int i = 0; i < numTags; ++i)
    {
        m_pose[msg->tag_ids[i]] = msg->poses[i];

        AprilTagProcessor::TAG_TYPE type = GetType(msg->tag_ids[i]);
        if (type == AprilTagProcessor::GOAL)
        {
            m_seesGoal = true;
        }
        else if(type == AprilTagProcessor::LANDMARK)
        {

        }
        else
        {
            ROS_ERROR("ERROR: Unkown tag type detected");
        }

        if (type != AprilTagProcessor::UNKNOWN)
        {
            // Don't worry about pausing if the robot is currently paused
            if (m_shouldPause == false)
            {
                if (type == AprilTagProcessor::LANDMARK)
                {
                    // Only need to update if it's been a while since the robot localized
                    if (m_pose[msg->tag_ids[i]].header.stamp - m_lastLocalizeTime > ros::Duration(9.0) ) {
                        //Check if the tag is closer than the threshold distance
                        if (GetDistance(m_pose[msg->tag_ids[i]]) < UPDATE_RANGE_THRESHOLD)
                        {
                            m_shouldPause = true;
                            m_seesLandmark = true;
                        }
                    }
                }
                else if (type == AprilTagProcessor::GOAL)
                {
                    // Only attempt to send the goal again if it's been a while since last sending it
                    if (m_pose[msg->tag_ids[i]].header.stamp - LastGoalSendTime(msg->tag_ids[i]) > ros::Duration(9.0) ) {
                        //The robot hasn't updated in a while...

                        //Check if the tag is closer than the threshold distance
                        if (GetDistance(m_pose[msg->tag_ids[i]]) < UPDATE_RANGE_THRESHOLD)
                        {
                            m_shouldPause = true;
                            m_seesGoal = true;
                        }
                    }
                }
            }
        }
    }

    m_lastImageTime = msg->poses[0].header.stamp;
}


/***********************************************************************
 *  Method: AprilTagProcessor::GetType
 *  Params: int tagID
 * Returns: AprilTagProcessor::TAG_TYPE
 * Effects:
 ***********************************************************************/
AprilTagProcessor::TAG_TYPE AprilTagProcessor::GetType(int tagID)
{
    if (m_goalTypeMap.find(tagID) != m_goalTypeMap.end())
    {
        return m_goalTypeMap[tagID];
    }
    return AprilTagProcessor::UNKNOWN;
}


/***********************************************************************
 *  Method: AprilTagProcessor::PosesDiffer
 *  Params: geometry_msgs::PoseWithCovariance &poseWithCovariance1, geometry_msgs::PoseWithCovariance &poseWithCovariance2
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool AprilTagProcessor::PosesDiffer(geometry_msgs::PoseWithCovariance &poseWithCovariance1, geometry_msgs::PoseWithCovariance &poseWithCovariance2)
{
    geometry_msgs::Pose pose1 = poseWithCovariance1.pose;
    geometry_msgs::Pose pose2 = poseWithCovariance2.pose;

    double diffX = abs(pose1.position.x - pose2.position.x);
    double diffY = abs(pose1.position.x - pose2.position.x);
    double diffTheta = abs(pose1.orientation.z - pose2.orientation.z);

    //Check linear distance
    static const double considerableDistance = 0.15; //15 cm difference is too much
    if (sqrt(diffX*diffX + diffY*diffY) > considerableDistance)
        return true;

    //Check difference in theta
    static const double considerableDifferenceTheta = 0.26; //15 degrees is too much
    if (diffTheta > considerableDifferenceTheta)
        return true;

    return false;
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
        ros::Time lastSeen = LastSeenTime(i);
        if (lastSeen != ros::Time(0) && lastSeen != m_lastImageTime)
        {
            // ROS_ERROR_STREAM("ERROR: Tag ID ("<<i<<") is not in view of the camera right now");
            continue;
        }

        return true;
    }
}


/***********************************************************************
 *  Method: AprilTagProcessor::PrintTransform
 *  Params: tf::StampedTransform &transform
 * Returns: void
 * Effects:
 ***********************************************************************/
void AprilTagProcessor::PrintTransform(tf::StampedTransform &transform)
{
    ROS_INFO_STREAM("Time: "<<transform.stamp_<<" | Parent: "<<transform.frame_id_<<" | Child?: "<<transform.child_frame_id_<<"\nTransform:");
    tf::Matrix3x3 mat = transform.getBasis();
    for (int i=0 ; i<3; i++)
    {
        std::cout << "[";
        for (int j=0; j<3; j++)
        {
            std::cout<<mat[i][j]<<" ";
        }
        std::cout << transform.getOrigin()[i] << "]" << std::endl;
    }
    std::cout << "]";
}


/***********************************************************************
 *  Method: AprilTagProcessor::GetDistance
 *  Params: tf::StampedTransform &tf
 * Returns: double
 * Effects:
 ***********************************************************************/
double AprilTagProcessor::GetDistance(tf::StampedTransform &tf)
{
    return sqrt(tf.getOrigin()[0]*tf.getOrigin()[0] + tf.getOrigin()[1]*tf.getOrigin()[1]);
}


/***********************************************************************
 *  Method: AprilTagProcessor::GetTagFrameName
 *  Params: int tagID
 * Returns: std::string
 * Effects:
 ***********************************************************************/
std::string AprilTagProcessor::GetTagFrameName(int tagID)
{
    if (GetType(tagID) == AprilTagProcessor::LANDMARK)
    {
        std::stringstream ss;
        ss << "april_tag["<<tagID<<"]";
        return ss.str();
    }
    return NULL;
}


/***********************************************************************
 *  Method: AprilTagProcessor::GetLandmarkFrameName
 *  Params: int tagID
 * Returns: std::string
 * Effects: Returns the frame name of the tag
 ***********************************************************************/
std::string AprilTagProcessor::GetLandmarkFrameName(int tagID)
{
    if (GetType(tagID) == AprilTagProcessor::LANDMARK)
    {
        std::stringstream ss;
        ss << "landmark["<<tagID<<"]";
        return ss.str();
    }
    return NULL;
}


/***********************************************************************
 *  Method: AprilTagProcessor::GetIDFromFrameName
 *  Params: std::string frame_name
 * Returns: int
 * Effects:
 ***********************************************************************/
int AprilTagProcessor::GetIDFromFrameName(std::string frame_name)
{
    char str[16];
    int id;
    sscanf(frame_name.c_str(), "%s[%d]", str, &id);
    return id;
}


/***********************************************************************
 *  Method: AprilTagProcessor::GetMapTransform
 *  Params: std::string frame_name, tf::StampedTransform &tf
 * Returns: bool
 * Effects: gives you the location of a frame in map coordinates,
 * if possible
 ***********************************************************************/
bool AprilTagProcessor::GetMapTransform(std::string frame_name, tf::StampedTransform &tf, ros::Time time)
{
    if (GetTransform("map", frame_name.c_str(), tf))
        return true;
    else
        return false;
}


/***********************************************************************
 *  Method: AprilTagProcessor::GetTransform
 *  Params: std::string frame_name1, std::string frame_name2, tf::StampedTransform &tf
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool AprilTagProcessor::GetTransform(std::string frame_name1, std::string frame_name2, tf::StampedTransform &tf, ros::Time time)
{
    if (m_tfListener->canTransform(frame_name1, frame_name2, time))
    {
        m_tfListener->lookupTransform(frame_name1, frame_name2, time, tf);
    }
    else
    {
        return false;
    }
}

/***********************************************************************
 *  Method: AprilTagProcessor::SetShouldPause
 *  Params: bool shouldPause
 * Returns: bool
 * Effects:
 ***********************************************************************/
bool AprilTagProcessor::SetShouldPause(bool shouldPause)
{
    m_shouldPause = shouldPause;
    return m_shouldPause;
}


/***********************************************************************
 *  Method: AprilTagProcessor::GetDistance
 *  Params: geometry_msgs::PoseStamped &pose
 * Returns: double
 * Effects:
 ***********************************************************************/
double AprilTagProcessor::GetDistance(geometry_msgs::PoseStamped &pose)
{
    return sqrt(pose.pose.position.x * pose.pose.position.x + pose.pose.position.y*pose.pose.position.y);
}

/***********************************************************************
 *  Method: AprilTagProcessor::LastGoalSendTime
 *  Params: int tagID
 * Returns: ros::Time
 * Effects:
 ***********************************************************************/
ros::Time AprilTagProcessor::LastGoalSendTime(int tagID)
{
    if (m_goalSendTime.find(tagID) != m_goalSendTime.end())
    {
        return m_goalSendTime[tagID];
    }
    return ros::Time(0);
}


