cmake_minimum_required(VERSION 3.0.2)
project(nautilus_scripts)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(DEPENDS
    rospy
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

# Subway car
catkin_install_python(PROGRAMS
  scripts/subway_car/subway_car_runner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Coral bleaching
catkin_install_python(PROGRAMS
  scripts/coral_bleaching/coral_bleaching_runner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
