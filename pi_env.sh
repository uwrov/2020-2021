#!/bin/sh

export PYTHONPATH=$PYTHONPATH:/home/pi/MIPI_Camera/RPI/python:/home/pi/RaspberryPi/Multi_Camera_Adapter/Multi_Adapter_Board_4Channel/Multi_Camera_Adapter_V2.2_python
export ROS_IP=192.168.1.19
export ROS_MASTER_URI=http://192.168.1.12:11311
export ROSLAUNCH_SSH_UNKNOWN=1
. /home/pi/2020-2021/devel/setup.sh
exec "$@"