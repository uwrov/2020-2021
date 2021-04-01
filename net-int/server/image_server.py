#!/usr/bin/env python3
import json
import rospy
from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit
import base64
from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
import cv2

from sensor_msgs.msg import CompressedImage
import numpy as np
import time

HOST_IP = "localhost"
HOST_PORT = "4040"

app = Flask(__name__)
sio = SocketIO(app, cors_allowed_origins="*")

image_subscriber = None
br = None

topics = {
    "/image/distribute": 0,
    "/nautilus/nautilus/camera1/nautilus_cam/compressed": 1,
    "/nautilus/nautilus/camera2/nautilus_cam/compressed": 2
}

time_between = time.process_time()

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
    global time_between
    print("Time between: ", "{:.2e}".format(time.process_time()-time_between))
    time_between = time.process_time()
    t0 = time.process_time()
    image = np.fromstring(data.data, np.uint8)
    t1 = time.process_time()
    # img = cv2.imdecode(image, cv2.IMREAD_COLOR)
    # retval, buffer = cv2.imencode('.png', image)
    img = base64.b64encode(image)
    t2 = time.process_time()
    sio.emit("Image Display", {'image': img, 'id': id}, broadcast = True)
    t3 = time.process_time()
    print("Elapsed 1: ", "{:.2e}".format(t1-t0))
    print("Elapsed 2: ", "{:.2e}".format(t2-t1))
    print("Elapsed 3: ", "{:.2e}".format(t3-t2))
    print("------")

"""
Time between:  9.95e-06
Elapsed 1:  1.64e-05
Elapsed 2:  4.39e-05
Elapsed 3:  3.11e-04
"""
"""
NEXT STEPS:
    - Check if Topics map needs to be initalized or not.
    - Copy and modify run.sh for 2020-2021.
    - Do further documentation if needed.
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

        image_subscriber = rospy.Subscriber("/image/distribute", Image, send_image, 0) # change chatter to url dest
        # front_cam_subscriber = rospy.Subscriber("/nautilus/nautilus/camera1/nautilus_cam", Image, send_image)
        front_cam_subscriber = rospy.Subscriber("/nautilus/nautilus/camera1/nautilus_cam/compressed", CompressedImage, send_image, 1)
        downward_cam_subscriber = rospy.Subscriber("/nautilus/nautilus/camera2/nautilus_cam/compressed", CompressedImage, send_image, 2)
        # br = CvBridge()
        sio.run(app, host=HOST_IP, port=HOST_PORT)
    except rospy.ROSInterruptException: pass
