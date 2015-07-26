cmake_minimum_required(VERSION 2.8.3)
project(moveit_r2_constraint_sampler)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  moveit_r2_kinematics
)

catkin_package(
  LIBRARIES moveit_r2_constraint_sampler moveit_r2_joint_constraint_sampler
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    moveit_core
    pluginlib
    moveit_r2_kinematics
)

include_directories(include
                    ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

add_library(moveit_r2_constraint_sampler src/r2_constraint_sampler.cpp)
add_library(moveit_r2_joint_constraint_sampler src/r2_joint_constraint_sampler.cpp)
target_link_libraries(moveit_r2_constraint_sampler ${catkin_LIBRARIES})
target_link_libraries(moveit_r2_joint_constraint_sampler ${catkin_LIBRARIES})

install(TARGETS moveit_r2_constraint_sampler moveit_r2_joint_constraint_sampler LIBRARY DESTINATION lib)
install(FILES moveit_r2_constraint_sampler_plugin_description.xml moveit_r2_joint_constraint_sampler_plugin_description.xml 
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
