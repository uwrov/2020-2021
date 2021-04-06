#!/usr/bin/env python3
import json
import rospy
from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit
import base64
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

HOST_IP = "localhost"
HOST_PORT = "4040"

app = Flask(__name__)
sio = SocketIO(app, cors_allowed_origins="*")

image_subscriber = None
br = None

# CD into the directory src/wb_sol/urdf
# roslaunch gazebo_ros empty_world.launch
# rosrun gazebo_ros spawn_model -file wb.urdf -urdf -model wheely_boi
# To run server
# rosrun wb_sol 2020_server.py
# source devel/setup.sh

def send_image(data, id):
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

    if id == 1 or id == 2:
        buffer = np.fromstring(data.data, np.uint8)
    else:
        image = br.imgmsg_to_cv2(data)
        retval, buffer = cv2.imencode('.png', image)
    img = base64.b64encode(buffer)
    sio.emit("Image Display", {'image': img, 'id': id}, broadcast = True)


"""
Time between:  9.95e-06
Elapsed 1:  1.64e-05
Elapsed 2:  4.39e-05
Elapsed 3:  3.11e-04
"""
"""
The cameras on nautilus publish sensor_msgs/Image to:

/nautilus/nautilus/camera1/nautilus_cam: Front facing camera
/nautilus/nautilus/camera2/nautilus_cam: Downward facing camera
"""
"""
1. adjust cv2
2. adjust rate at which we send images
3. adjust sensors width, height, update rate
"""
if __name__ == '__main__':
    """ Sets up rospy and starts server """
    try:
        print("image server is running")
        rospy.init_node('wheely_boi', anonymous=True)

        image_subscriber = rospy.Subscriber("/image/distribute", Image, send_image, 3) # change chatter to url dest
        front_cam_subscriber = rospy.Subscriber("/nautilus/nautilus/camera1/nautilus_cam/compressed", CompressedImage, send_image, 1)
        downward_cam_subscriber = rospy.Subscriber("/nautilus/nautilus/camera2/nautilus_cam/compressed", CompressedImage, send_image, 2)

        br = CvBridge()
        sio.run(app, host=HOST_IP, port=HOST_PORT)
    except rospy.ROSInterruptException: pass
