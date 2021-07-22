#!/bin/sh

export ROS_IP=192.168.0.12
export ROS_MASTER_URI=http://localhost:11311
export ROSLAUNCH_SSH_UNKNOWN=1
. /home/uwrov/2020-2021/devel/setup.sh
exec "$@"