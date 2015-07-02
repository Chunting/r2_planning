cmake_minimum_required(VERSION 2.8.3)
project(moveit_ompl_pose_constraint_interface)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


find_package(Boost REQUIRED system filesystem date_time thread serialization)
find_package(catkin REQUIRED COMPONENTS
    ompl_source
    moveit_ompl_planning_interface
    moveit_r2_kinematics
    moveit_ompl_components
)

catkin_package(
  LIBRARIES
    moveit_ompl_pose_constraint_interface
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    ompl_source
    moveit_ompl_planning_interface
    moveit_r2_kinematics
    moveit_ompl_components
)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    SYSTEM)

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})


# Planning for any planner with a single pose constraint and fancy interpolation
add_library(moveit_ompl_pose_constraint_interface
            src/GeometricPoseConstraintPlanningContext.cpp
            )
target_link_libraries(moveit_ompl_pose_constraint_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LIBFCL_LIBRARIES})

install (TARGETS moveit_ompl_pose_constraint_interface
         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)

install(FILES ompl_pose_constraint_planning_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
