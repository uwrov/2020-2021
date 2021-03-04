cmake_minimum_required(VERSION 2.8.3)
project(nautilus_worlds)

find_package(catkin REQUIRED COMPONENTS gazebo_ros)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uuv_gazebo_worlds
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

install(DIRECTORY launch models
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)