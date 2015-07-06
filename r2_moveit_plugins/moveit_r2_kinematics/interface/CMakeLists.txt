cmake_minimum_required(VERSION 2.8.3)
project(r2_tree_kinematics)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  tf_conversions
)

catkin_package(
  LIBRARIES r2_tree_kinematics
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning
    tf_conversions
)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    )

link_directories(${catkin_LIBRARY_DIRS})

add_library(r2_tree_kinematics src/r2_tree_kinematics_interface.cpp
                               src/nasa_robodyn/KdlTreeIk.cpp
                               src/nasa_robodyn/KdlTreeParser.cpp
                               src/nasa_robodyn/KdlTreeTr.cpp
                               src/nasa_robodyn/KdlTreeUtilities.cpp
                               src/nasa_robodyn/MotionLimiter.cpp
                               src/nasa_robodyn/MobileTreeIk.cpp
                               src/nasa_robodyn/KdlTreeFk.cpp)
target_link_libraries(r2_tree_kinematics ${catkin_LIBRARIES})

install (TARGETS ${PROJECT_NAME}
         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
