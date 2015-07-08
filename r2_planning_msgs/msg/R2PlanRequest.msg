# This message is modeled after moveit_msgs/MotionPlanRequest

# The name of the group to plan for
string group_name

# The joint positions for the initial state of the robot
moveit_msgs/RobotState start_state

# The possible goal states for the model to plan for. Each element of
# the array defines a goal region. The goal is achieved
# if the constraints for a particular region are satisfied
moveit_msgs/Constraints[] goal_constraints

# No state at any point along the path in the produced motion plan will violate
# these constraints (this applies to all points, not just waypoints)
moveit_msgs/Constraints path_constraints

# The maximum time allowed for planning (seconds)
float64 allowed_planning_time

# The name of the motion planner to use. If no name is specified,
# a default motion planner will be used
string planner_id

# The number of times this plan is to be computed. Shortest solution
# will be reported.  Computation will be parallelized up to
# maximum_number_threads (rqt) or num_planning_attempts, whichever is
# less.  The default is 1.
int32 num_planning_attempts

# A set of updates to the AllowedCollisionMatrix.  Use this to enable/
# disable collisions with handrails or other objects in the scene.
CollisionUpdate[] collision_updates