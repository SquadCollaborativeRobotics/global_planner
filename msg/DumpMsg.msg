#id number
int32 id

#time created
time time

#poses
geometry_msgs/Pose pose1
geometry_msgs/Pose pose2

#robots assigned
int32 robotID1
int32 robotID2

#status = 0 if finished, 1 if available, 2 if in progress, 3 if finished
int32 status