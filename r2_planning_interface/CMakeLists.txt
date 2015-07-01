cmake_minimum_required(VERSION 2.8.3)
project(r2_planning_interface)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# In ROS Hydro, MoveIt links against version 2 of Boost Filesystem
# Force everything to use this version to prevent compile error
if ($ENV{ROS_DISTRO} STREQUAL "hydro")
  message("Explicitly linking against Boost.Filesystem v2")
  add_definitions(-DUSE_BOOST_FILESYSTEM_V2)
endif()

find_package(catkin COMPONENTS
  roscpp
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_move_group
  moveit_ros_warehouse
  #nasa_robodyn
  #moveit_r2_kinematics
  r2_tree_kinematics
  moveit_ros_planning_interface
  REQUIRED)

# catkin_package defines the env vars used in install later
catkin_package(
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp moveit_msgs moveit_core moveit_ros_planning moveit_ros_move_group moveit_ros_warehouse moveit_ros_planning_interface r2_tree_kinematics
   DEPENDS
   INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
)

include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR}/include)
link_directories(${catkin_LIBRARY_DIRS})
add_library(${PROJECT_NAME} src/R2Interface.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
