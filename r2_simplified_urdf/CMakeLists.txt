cmake_minimum_required(VERSION 2.8.3)
project(r2_simplified_urdf)

find_package(catkin REQUIRED)

catkin_package()

install(FILES
  r2c6_legs_only_creepy.xacro
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})