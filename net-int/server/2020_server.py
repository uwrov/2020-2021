#!/usr/bin/env python3
import json
import rospy
from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit
from geometry_msgs.msg import Twist
# image stuff
import base64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

HOST_IP = "localhost"
HOST_PORT = "4040"

app = Flask(__name__)
sio = SocketIO(app, cors_allowed_origins="*")

velocity_publisher = None
rate = None
image_subsciber = None
<<<<<<< HEAD
br = None
=======
image = None
>>>>>>> 3a6e8f3e4450dc148d75f7b1bf5be818f4ab21e5
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
    msg.linear.x = state["lin_x"]
    msg.linear.y = state["lin_y"]
    msg.linear.z = state["lin_z"]
    msg.angular.x = state["lin_x"]
    msg.angular.y = state["lin_y"]
    msg.angular.z = state["lin_z"]
    #while not rospy.is_shutdown():
    rospy.loginfo("Sending Command v:" + str(msg.linear.x))
    velocity_publisher.publish(msg)
    rate.sleep()
    #rospy.signal_shutdown('task done')


def send_image(data):
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
    rospy.loginfo('Image received...')
    image = br.imgmsg_to_cv2(data)
    retval, buffer = cv2.imencode('.png', image)
    img = base64.b64encode(buffer)
<<<<<<< HEAD
    sio.emit("Image Display", {'image': img}, broadcast = True)
=======
    emit("Image Detection", {'image': img}, broadcast = True)
>>>>>>> 3a6e8f3e4450dc148d75f7b1bf5be818f4ab21e5

# def send_sensor_data():
#     emit('Senor Data', {'sensor data': sensor_data}, broadcast=True)
#
# @sio.on('Send Command')
# def send_command(command):
#     #emit('Command', command, broadcast=True)
#     print(hello)

if __name__ == '__main__':
    """ Sets up rospy and starts server """
    try:
        rospy.init_node('wheely_boi', anonymous=True)
<<<<<<< HEAD
        #velocity_publisher = rospy.Publisher('/wheely_boi/wheely_boi/cmd', Twist, queue_size=10)
        image_subsciber = rospy.Subscriber("/image/distribute", Image, send_image) # change chatter to url dest
        br = CvBridge()
        #rate = rospy.Rate(10)
=======
        velocity_publisher = rospy.Publisher('/wheely_boi/wheely_boi/cmd', Twist, queue_size=10)
        image_subsciber = rospy.Subscriber("chatter", Image, send_image) # change chatter to url dest
        br = CvBridge()
        rate = rospy.Rate(10)
>>>>>>> 3a6e8f3e4450dc148d75f7b1bf5be818f4ab21e5
        sio.run(app, host=HOST_IP, port=HOST_PORT)
    except rospy.ROSInterruptException: pass
