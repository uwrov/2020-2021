cmake_minimum_required(VERSION 3.0.2)
project(camera_stream)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(DEPENDS
  rospy
)

catkin_python_setup()

catkin_install_python(PROGRAMS
scripts/picam_stream.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
scripts/usbcam_pub.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
scripts/switchable_cams.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
scripts/multi_stream.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

