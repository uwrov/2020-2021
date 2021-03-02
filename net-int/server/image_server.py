#!/usr/bin/env python3
import json
import rospy
from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit
import base64
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2

import timer


HOST_IP = "localhost"
HOST_PORT = "4040"

app = Flask(__name__)
sio = SocketIO(app, cors_allowed_origins="*")

image_subsciber = None
br = None

# CD into the directory src/wb_sol/urdf
# roslaunch gazebo_ros empty_world.launch
# rosrun gazebo_ros spawn_model -file wb.urdf -urdf -model wheely_boi
# To run server
# rosrun wb_sol 2020_server.py
# source devel/setup.sh

def send_image(data):
    """
    Sends image to client

    Receives movement information from ROS as a image message.
    The image is then converted into a cv2 image and then encoded in base 64 so
    it can be sent as a JSON object to the client via socket.io

    Parameters
    -------
    data : ROS image
        stores image from ROS

    Returns
    -------
    None
    """
    # rospy.loginfo('Image received...')
    image = np.fromstring(data.data, np.uint8)
    # img = cv2.imdecode(image, cv2.IMREAD_COLOR)
    # retval, buffer = cv2.imencode('.png', image)
    img = base64.b64encode(image)
    sio.emit("Image Display", {'image': img}, broadcast = True)
    # rospy.loginfo('Emitting Image')

if __name__ == '__main__':
    """ Sets up rospy and starts server """
    try:
        print("image server is running")
        rospy.init_node('wheely_boi', anonymous=True)
        image_subsciber = rospy.Subscriber("/nautilus/nautilus/camera1/nautilus_cam/compressed", CompressedImage, send_image) # change chatter to url dest
        # br = CvBridge()
        sio.run(app, host=HOST_IP, port=HOST_PORT)
    except rospy.ROSInterruptException: pass
