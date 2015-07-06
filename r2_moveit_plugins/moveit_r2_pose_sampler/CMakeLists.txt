cmake_minimum_required(VERSION 2.8.3)
project(moveit_r2_pose_sampler)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  moveit_r2_kinematics
)

catkin_package(
  LIBRARIES moveit_r2_pose_sampler
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    moveit_core
    pluginlib
    moveit_r2_kinematics
)

include_directories(include
                    ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

add_library(moveit_r2_pose_sampler src/r2_pose_sampler.cpp)
target_link_libraries(moveit_r2_pose_sampler ${catkin_LIBRARIES})

install (TARGETS ${PROJECT_NAME}
         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES moveit_r2_pose_sampler_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})