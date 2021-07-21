#!/bin/bash

echo "Hello World!"

# echo "launching Nautilus"
# gnome-terminal -- bash -c 'source devel/setup.sh; roslaunch nautilus_worlds nautilus_sim.launch reset_tam:=true prop:=subway_car; $SHELL'
# sleep 40s
#
# echo "starting roscore"
# gnome-terminal -- bash -c 'source devel/setup.sh; roscore; $SHELL'
# sleep 3s
echo "starting client"
gnome-terminal -- bash -c 'cd net-int/reactgui; npm start; $SHELL'
sleep 30s
# # #

echo "starting main server"
gnome-terminal -- bash -c 'source devel/setup.sh; cd net-int/server; python3 main_server.py; $SHELL'
sleep 5s

# echo "starting image server"
# gnome-terminal -- bash -c 'source devel/setup.sh; cd net-int/server; python3 image_server.py; $SHELL'
# sleep 5s
#
# #echo "starting image client"
# #gnome-terminal -- bash -c 'source devel/setup.sh; cd net-int/server; python3 image_client.py; $SHELL'
# #sleep 5s
# #
# echo "starting movement server"
# gnome-terminal -- bash -c 'source devel/setup.sh; cd src/nautilus_control/scripts; rosrun nautilus_control Move_Server.py; $SHELL'
# sleep 3s

echo "All servers are running"
