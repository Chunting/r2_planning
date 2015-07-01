cmake_minimum_required(VERSION 2.8.3)
project(r2_rviz_planning_interface)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Qt Stuff
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

find_package(catkin COMPONENTS
  roscpp
  rviz
  r2_planning_interface
  #interactive_markers
  pluginlib
  REQUIRED)

# catkin_package defines the env vars used in install later
catkin_package(
   LIBRARIES r2_rviz_planning_interface
   CATKIN_DEPENDS roscpp rviz pluginlib r2_planning_interface #interactive_markers
   DEPENDS
   INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
)

include_directories(${catkin_INCLUDE_DIRS} ${QT_INCLUDE_DIR} ${CMAKE_CURRENT_LIST_DIR}/include)
link_directories(${catkin_LIBRARY_DIRS})

# QT MOC pre-processor for object headers
set(QT_MOC_HEADERS include/r2_rviz_planning_interface/RVizPlanningInterface.h)
qt4_wrap_cpp(MOC_SOURCES ${QT_MOC_HEADERS})

add_library(r2_rviz_planning_interface src/RVizPlanningInterface.cpp ${MOC_SOURCES})
target_link_libraries(r2_rviz_planning_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${QT_LIBRARIES})

install(FILES
  r2_rviz_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
