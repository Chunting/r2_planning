cmake_minimum_required(VERSION 2.8.3)
project(moveit_ompl_components)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


find_package(Boost REQUIRED system filesystem date_time thread serialization)
find_package(catkin REQUIRED COMPONENTS
    ompl_source
    fcl
    moveit_ompl_planning_interface
)

catkin_package(
  LIBRARIES
    moveit_ompl_components
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    ompl_source
    fcl
    moveit_ompl_planning_interface
)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    SYSTEM)

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# Library with a collection of OMPL components useful for planning
add_library(moveit_ompl_components
            src/SubdivisionMotionValidator.cpp
            src/LinearMotionValidator.cpp
            src/CartesianDistanceOptimizationObjective.cpp
            src/GoalSamplingPathSimplifier.cpp
            src/planners/LazyTRRT.cpp
            src/planners/CASPER.cpp
            src/planners/HiLoPoseDecomposition.cpp
            src/planners/HiLo.cpp
            src/planners/BiHiLo.cpp
            src/CartesianSpaceInterpolator.cpp
            src/CartesianDistancePathSimplifier.cpp
            )
target_link_libraries(moveit_ompl_components ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LIBFCL_LIBRARIES})

install (TARGETS moveit_ompl_components
         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)

