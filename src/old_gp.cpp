#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/thread/mutex.hpp>
#include <global_planner/GarbageCan.h>
#include <global_planner/SoundMsg.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

// The global planner uses the actionlib to do the following:
// Start in safe mode
//
// In safe state
//  cancel all goals and no april tag callbacks
//  if start command:
//   Transition to search state
//  if goto state command:
//   Transition to given state
//
// In search state
//  travel to poses A,B,etc... on known map
//  If a trashcan is found at any time:
//   Transition to approach_trash state
//  else if hit final search state:
//   Transition to end state
//
// In approach_trash state
//  travel to infront of trashcan
//  if reach trash pose:
//   transition to dump trash state
//
// In dump trash state
//  travel to dump trash position
//  if reach dump trash position:
//   transition to end state
//
// In end state
//  travel to final pose
//  if reaach final pose:
//   transition to safe state


// Convenience Typedef
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

boost::shared_ptr<MoveBaseClient> action_client_ptr;

// April tag subscriber
ros::Subscriber sub;
ros::Publisher cmd_pub;
ros::Publisher sound_pub;

// State Machine for Fall Demo #2
enum State {
  SAFE = 0,
  SEARCH_A = 1,
  SEARCH_B = 2,
  SEARCH_C = 3,
  SEARCH_D = 4,
  SEARCH_E = 5,
  APPROACH_TRASH = 6,
  DUMP_TRASH = 7,
  END = 8,
  PAUSE = 99,
  RESUME = 100,
  START = 999,
};
State currState = START;
State state_before_pause = SAFE;
ros::Time timeofpause;

// Command value read from topic, or on transition to SAFE
int command_value = -1;

bool given_start_state = false;

// Search poses
struct search_pose
{
  // x, y, rot
  double x, y, rz, rw;
};
search_pose search_poses[] = { {1, -0.8, 0, 1},
                               {1.45, 1.0, 1, 0},
                               {-2, .85, -.73, .68},
                               {-1.8, -0.8, 0, 1},
                               {0, -1, 0.67, 0.72}
                             };
// End position
search_pose end_pose = {0, 0, 0, 1};

void send_sound_num(std::string str, int num_times)
{
  global_planner::SoundMsg s;
  s.filename = str;
  s.num_times = num_times;
  s.text_output = std::string();
  sound_pub.publish(s);
  ros::spinOnce();
}

void send_sound(std::string str)
{
  send_sound_num(str, 1);
}


// Sets given goal to given x,y and rotation quat rz,rw
void setGoalPoseRaw(double x, double y, double rz, double rw,
                 move_base_msgs::MoveBaseGoal &goal) {
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.z = rz;
  goal.target_pose.pose.orientation.w = rw;
}
void setGoalPose(const search_pose &s,
                 move_base_msgs::MoveBaseGoal &goal) {
  setGoalPoseRaw(s.x, s.y, s.rz, s.rw, goal);
}

// Given a trashcan as a PoseStamped messge, returns the goal pose in front of it
void getGoalPoseFromTrashcan(const geometry_msgs::PoseStamped& msg,
                             move_base_msgs::MoveBaseGoal &goal) {
  double theta = 2 * acos(msg.pose.orientation.w);
  setGoalPoseRaw(msg.pose.position.x - 0.5*cos(theta),
                 msg.pose.position.y - 0.5*sin(theta),
                 msg.pose.orientation.z,
                 msg.pose.orientation.w,
                 goal);
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
}

bool found_trashcan = false;

// Callback for new april tags
void trashcanTagSearcherCallback(const global_planner::GarbageCan::ConstPtr& msg) {
  // ROS_INFO("April Tag Callback!");

  geometry_msgs::PoseStamped pose = msg->pose;
  int can_num = msg->can_num;

   ROS_INFO("Can number: %d", msg->can_num);

  // If april tag is id 6 (trashcan)
  if (can_num == 6) {
    // ROS_INFO("Header Match!");
    found_trashcan = true;

    move_base_msgs::MoveBaseGoal goal;

    // Set goal position to in front of april tag
    getGoalPoseFromTrashcan(pose, goal);

    // http://mirror.umd.edu/roswiki/doc/diamondback/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html#a6bdebdd9f43a470ecd361d2c8b743188
    // If a previous goal is already active when this is called.
    // We simply forget about that goal and start tracking the new goal.
    // No cancel requests are made.
    ROS_INFO("TRASH GOAL SENT!");
    action_client_ptr->sendGoal(goal);
  }
}

void transition(State state, ros::NodeHandle &n) {
  //Only transition if it's new
  if (currState != state) {
    State lastState = currState; //used when pausing
    currState = state;

    ROS_INFO("-- STATE TRANSITION %d --", currState);

    int chosen_search_pose = -1;
    switch(currState) {
      case SAFE:
      action_client_ptr->cancelAllGoals(); // Cancel any current move goals
      sub.shutdown();
      command_value = 0;
      break;

      case SEARCH_A:
      chosen_search_pose = 0;
      ROS_INFO("SENDING POSE SEARCH_A to /move_base/goal");
      break;

      case SEARCH_B:
      chosen_search_pose = 1;
      ROS_INFO("SENDING POSE SEARCH_B to /move_base/goal");
      break;

      case SEARCH_C:
      chosen_search_pose = 2;
      ROS_INFO("SENDING POSE SEARCH_C to /move_base/goal");
      break;

      case SEARCH_D:
      chosen_search_pose = 3;
      ROS_INFO("SENDING POSE SEARCH_D to /move_base/goal");
      break;

      case SEARCH_E:
      chosen_search_pose = 4;
      ROS_INFO("SENDING POSE SEARCH_E to /move_base/goal");
      break;

      case APPROACH_TRASH:
      send_sound("approaching_trash");
      sub.shutdown();
      break;

      case DUMP_TRASH:
      sub.shutdown();
      {
        // Set goal to search pose
        move_base_msgs::MoveBaseGoal goal;
        setGoalPose(end_pose, goal);
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        // Send goal to action client
        action_client_ptr->sendGoal(goal);
        ROS_INFO("END GOAL SENT");
      }

      break;

      case END:
      sub.shutdown();
      break;

      case PAUSE:
      ROS_INFO("PAUSE received... cancel all goals");
      state_before_pause = lastState;
      timeofpause = ros::Time::now();
      action_client_ptr->cancelAllGoals(); // Cancel any current move goals
      sub.shutdown();
      break;

      case RESUME:
      ROS_ERROR("Interesting... we are trying to transition to 'resume'. weird");
      break;
    }

    std_msgs::Int32 cmd_msg;
    cmd_msg.data = currState;
    cmd_pub.publish(cmd_msg);
    ROS_INFO_STREAM("Sent command state: "<<cmd_msg.data);

    // If searching
    if (chosen_search_pose >= 0) {
      // Subscribe to trashcan searcher
      sub = n.subscribe("garbageCan", 10, trashcanTagSearcherCallback);

      // Set goal to search pose
      move_base_msgs::MoveBaseGoal goal;
      setGoalPose(search_poses[chosen_search_pose], goal);
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      // Send goal to action client
      action_client_ptr->sendGoal(goal);
      ROS_INFO("GOAL SENT");
    }
  }
}

void commandCallback(const std_msgs::Int32::ConstPtr& msg) {
  if (command_value != msg->data)
  {
    command_value = msg->data;
    ROS_INFO("Received command %d", command_value);
  }
}

int main(int argc, char** argv){
  // ROS Node Initialization
  ros::init(argc, argv, "global_planner");
  ros::NodeHandle n;

  // Subscribe to command node
  ros::Subscriber cmd_sub = n.subscribe("cmd_state", 10, commandCallback);
  // Publisher to send current state to the rest of the world when transition happens
  cmd_pub = n.advertise<std_msgs::Int32>("cmd_state", 10);

  // Publisher to send current state to the rest of the world when transition happens
  sound_pub = n.advertise<global_planner::SoundMsg>("play_sound", 10);
  ros::spinOnce();

  ROS_INFO_STREAM("Sending test sound");
  sleep(1);
  send_sound("beep.wav");

  action_client_ptr.reset( new MoveBaseClient("move_base", true) );

  // Create Rate Object for sleeping
  ros::Rate r(100);

  // Wait for the action server to come up
  while(ros::ok() && !action_client_ptr->waitForServer(ros::Duration(1.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }


  ROS_INFO("Starting global planner");

  while(ros::ok()){
    if (command_value == 0) {
      given_start_state = false;
      transition(SAFE, n);
    }
    if (found_trashcan) {
      found_trashcan = false;
      transition(APPROACH_TRASH, n);
    }

    // State machine command override from safe to chosen state
    switch(currState) {
      case START:
        if (command_value > 0 && command_value < 6)
        {
          given_start_state = true;
          for (int i=0; i<5; i++)
          {
            send_sound("beep.wav");
            sleep(1.5);
          }
          State s = static_cast<State>(command_value);
          transition(s, n);
        }
        break;
      case SAFE:
        switch (command_value) {
          State s;
          case SEARCH_A:
          case SEARCH_B:
          case SEARCH_C:
          case SEARCH_D:
          case SEARCH_E:
          s = static_cast<State>(command_value);
          transition(s, n);
          break;
          case APPROACH_TRASH:
          transition(APPROACH_TRASH, n);
          break;
          case DUMP_TRASH:
          transition(DUMP_TRASH, n);
          break;
          case 8:
          transition(END, n);
          break;
          case 99:
          transition(PAUSE, n);
          break;
          case 100:
          transition(RESUME, n);
        }
        break;


      case SEARCH_A:
        ROS_INFO_THROTTLE(2,"command_value = %d, Status : %s", command_value, action_client_ptr->getState().toString().c_str());
        if (command_value == PAUSE)
          transition(PAUSE, n);
        if (action_client_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
            command_value == 2) {
          send_sound("beep.wav");
          transition(SEARCH_B, n);
        }
        break;

      case SEARCH_B:
        ROS_INFO_THROTTLE(2,"command_value = %d, Status : %s", command_value, action_client_ptr->getState().toString().c_str());
        if (command_value == PAUSE)
          transition(PAUSE, n);
        if (action_client_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
            command_value == 3) {

          send_sound("beep.wav");
          transition(SEARCH_C, n);
        }
        break;

      case SEARCH_C:
        ROS_INFO_THROTTLE(2,"command_value = %d, Status : %s", command_value, action_client_ptr->getState().toString().c_str());
        if (command_value == PAUSE)
          transition(PAUSE, n);
        if (action_client_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
            command_value == 4) {
          send_sound("beep.wav");
          transition(SEARCH_D, n);
        }
        break;

      case SEARCH_D:
        ROS_INFO_THROTTLE(2,"command_value = %d, Status : %s", command_value, action_client_ptr->getState().toString().c_str());
        if (command_value == PAUSE)
          transition(PAUSE, n);
        if (action_client_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
            command_value == 5) {
          send_sound("beep.wav");
          transition(SEARCH_E, n);
        }
        break;

      case SEARCH_E:
        ROS_INFO_THROTTLE(2,"command_value = %d, Status : %s", command_value, action_client_ptr->getState().toString().c_str());
        if (command_value == PAUSE)
          transition(PAUSE, n);
        if (action_client_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
            command_value == 6) {
          send_sound("beep.wav");
          transition(DUMP_TRASH, n);
        }
        break;

      case APPROACH_TRASH:
        ROS_INFO_THROTTLE(2,"command_value = %d, Status : %s", command_value, action_client_ptr->getState().toString().c_str());
        if (command_value == PAUSE)
          transition(PAUSE, n);
        if (action_client_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
            command_value == 7) {
          send_sound("seen_goal.wav");
          transition(DUMP_TRASH, n);
        }
        break;

      // Dump trash means go to the end search_pos 6
      case DUMP_TRASH:
        ROS_INFO_THROTTLE(2,"command_value = %d, Status : %s", command_value, action_client_ptr->getState().toString().c_str());
        if (command_value == PAUSE)
          transition(PAUSE, n);
        if (action_client_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
            command_value == 8) {
          send_sound("beep.wav");
          transition(END, n);
        }
        break;

      case END:
        ROS_INFO_THROTTLE(2,"command_value = %d, Status : %s", command_value, action_client_ptr->getState().toString().c_str());
        send_sound("seen_goal.wav");
        ros::spinOnce();
        sleep(1);
        exit(1);
        break;

      case PAUSE:
        if (command_value == RESUME)
        {
          transition(state_before_pause, n);
        }
        if (ros::Time::now() - timeofpause > ros::Duration(10))
        {
          ROS_ERROR("We've been paused for too long. Let's resume anyways!");
          transition(state_before_pause,n);
        }
    }
    if (given_start_state && action_client_ptr->getState() == actionlib::SimpleClientGoalState::ABORTED) {
      transition(SAFE, n);
      send_sound("crash.wav");
    }

    // Update callbacks after the fact, for next loop iteration.
    ros::spinOnce();

    r.sleep();
  }
}