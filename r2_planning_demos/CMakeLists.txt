cmake_minimum_required(VERSION 2.8.3)
project(r2_planning_demos)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin COMPONENTS
  r2_planning_interface
  REQUIRED)

# catkin_package defines the env vars used in install later
catkin_package(
   LIBRARIES
   CATKIN_DEPENDS r2_planning_interface
   DEPENDS
   INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
)

include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR}/include)
link_directories(${catkin_LIBRARY_DIRS})

add_executable(iss_navigation src/iss_navigation.cpp)
target_link_libraries(iss_navigation ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(argos_demo src/argos_demo.cpp)
target_link_libraries(argos_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(localizer src/localizer.cpp)
target_link_libraries(localizer ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(executor src/executor.cpp)
target_link_libraries(executor ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(spawn_world src/spawn_world.cpp)
target_link_libraries(spawn_world ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(r2_planning_server src/planning_server.cpp)
target_link_libraries(r2_planning_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install (TARGETS iss_navigation argos_demo localizer executor spawn_world r2_planning_server
         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY world DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})