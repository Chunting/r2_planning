cmake_minimum_required(VERSION 2.8.3)
project(r2_benchmark)

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
  r2_planning_interface
  moveit_benchmarks
  REQUIRED)

# catkin_package defines the env vars used in install later
catkin_package(
   LIBRARIES
   CATKIN_DEPENDS moveit_benchmarks #r2_planning_interface
   DEPENDS
   INCLUDE_DIRS
)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(iss_climbing_benchmark src/iss_climbing_benchmark.cpp)
target_link_libraries(iss_climbing_benchmark ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(iss_climbing_benchmark_arms src/iss_climbing_benchmark_arms.cpp)
target_link_libraries(iss_climbing_benchmark_arms ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(make_db src/make_database.cpp)
target_link_libraries(make_db ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install (TARGETS iss_climbing_benchmark make_db
         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY benchmark DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})