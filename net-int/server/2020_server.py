#!/usr/bin/env python3
import json
import rospy
from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit
from geometry_msgs.msg import Twist


velocity_publisher = None
rate = None

current = None

# CD into the directory src/wb_sol/urdf
# roslaunch gazebo_ros empty_world.launch
# rosrun gazebo_ros spawn_model -file wb.urdf -urdf -model wheely_boi
# To run server
# rosrun wb_sol 2020_server.py
# source devel/setup.sh

@sio.on("Send State")
def send_state(state):
    """
    Sends contoller input to rospy

    Receives movement information from controller as a JSON object.
    The movement info is then converted into a Twist object and published
    to the rospy.

    Parameters
    -------
    state : JSON/Dictionary
        stores the movement of the controller in terms of linear components and
        anglular components.
        state = {lin_x: 10, lin_y: 0, lin_z: 0, ang_x: 0, ang_y: 0, ang_z: 3}

    Returns
    -------
    None
    """
    msg = Twist()
    if current is None or msg.linear.x != current['lin_x'] or msg.linear.y != current['lin_y'] or msg.linear.y != current['lin_y'] or msg.angular.x != current['ang_x'] or msg.angular.y != current['ang_y']
    or msg.angular.z != current['ang_z']:

	 msg.linear.x = state["lin_x"]
	 msg.linear.y = state["lin_y"]
	 msg.linear.z = state["lin_z"]
	 msg.angular.x = state["ang_x"]
	 msg.angular.y = state["ang_y"]
	 msg.angular.z = state["ang_z"]
         current.linear.x = state["lin_x"]
	 current.linear.y = state["lin_y"]
	 current.linear.z = state["lin_z"]
	 current.angular.x = state["ang_x"]
	 current.angular.y = state["ang_y"]
	 current.angular.z = state["ang_z"]  

    #while not rospy.is_shutdown():
    rospy.loginfo("Sending Command v:" + str(current.linear.x))
    velocity_publisher.publish(current)
    rate.sleep()
    #rospy.signal_shutdown('task done')

#def send_image(data):
def send_image():
    """
    Sends image to client

    Receives movement information from ROS as a image message.
    The image is then converted into a cv2 image and then encoded in vase 64 so
    it can be sent as a JSON object to the client via socket.io

    Parameters
    -------
    data : ROS image
        stores image from ROS

    Returns
    -------
    None
    """
    #rospy.loginfo('Image received...')
    #image = br.imgmsg_to_cv2(data)
    image = cv2.imread("smile.png")
    retval, buffer = cv2.imencode('.png', image)
    img = base64.b64encode(buffer)
    sio.emit("Image Display", {'image': img}, broadcast = True)


# def send_sensor_data():
#     emit('Senor Data', {'sensor data': sensor_data}, broadcast=True)
#
# @sio.on('Send Command')
# def send_command(command):
#     #emit('Command', command, broadcast=True)
#     print(hello)

if __name__ == '__main__':
    """ Sets up rospy and starts server """
    # try:
    #     rospy.init_node('wheely_boi', anonymous=True)
    #     #velocity_publisher = rospy.Publisher('/wheely_boi/wheely_boi/cmd', Twist, queue_size=10)
    #     image_subsciber = rospy.Subscriber("/image/distribute", Image, send_image) # change chatter to url dest
    #     br = CvBridge()
    #     #rate = rospy.Rate(10)
    #     sio.run(app, host=HOST_IP, port=HOST_PORT)
    # except rospy.ROSInterruptException: pass
    sio.run(app, host=HOST_IP, port=HOST_PORT)
    send_image()
