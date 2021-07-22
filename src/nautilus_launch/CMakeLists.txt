cmake_minimum_required(VERSION 3.0.2)
project(nautilus_launch)

catkin_package()
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/launch_interface.bash
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})