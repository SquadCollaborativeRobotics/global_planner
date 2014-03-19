#status of the robot

#robot id
int32 id

#robot name
string name

#robot state
int32 state

#task id -> id of the task (waypoint, goal, dump) that the robot is running
int32 taskID

#pose of the robot
geometry_msgs/Pose pose

#velocity of the bot
geometry_msgs/Twist twist

#storage used
int32 storage_used

#storage capacity
int32 storage_capacity

#robot type
uint8 type
