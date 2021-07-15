# UWROV Server Directory
This is all of the servers that we will be working with for the 2020-2021 year.

// link to documentation

## Move Server
[Move_Server.py](./Move_Server.py) handles the movement for the ROV.

It utilizes `Wrench` objects from ROS, which stores a force and torque, to
indicate the desired velocity and sleep of the ROV.

## Image Server
[image_server.py](./image_Server.py) handles the camera feed of the ROV.
It receives image from ROS as a `compressed image` and then sends it to the client
as a `JSON` object

## Image Client

## Scripts
