# This message is modeled after moveit_msgs/MotionPlanResponse

# The state of the robot at the start of the trajectory
moveit_msgs/RobotState trajectory_start

# The group used for planning (usually the same as in the request);
string group_name

# The solution trajectory, if found
moveit_msgs/RobotTrajectory robot_trajectory

# The solution trajectory with additional information
# needed for visualization.
moveit_msgs/RobotTrajectory visualization_trajectory

# Planning time (seconds)
float64 planning_time

# Error code - encodes the overall reason for failure
moveit_msgs/MoveItErrorCodes error_code