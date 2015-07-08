cmake_minimum_required(VERSION 2.8.3)
project(r2_planning_msgs)

set(MSG_DEPS
  moveit_msgs)

find_package(catkin REQUIRED genmsg ${MSG_DEPS})

set(MSG_FILES
CollisionUpdate.msg
R2PlanRequest.msg
R2PlanResponse.msg
)

set(SRV_FILES
R2MotionPlanRequest.srv
R2SimplePlanRequest.srv
)

#add_action_files(DIRECTORY action FILES ${ACT_FILES})
add_message_files(DIRECTORY msg FILES ${MSG_FILES})
add_service_files(DIRECTORY srv FILES ${SRV_FILES})

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(DEPENDS ${MSG_DEPS})
