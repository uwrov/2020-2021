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

# HOST_IP = "localhost"
HOST_IP = "0.0.0.0"
HOST_PORT = "4040"

app = Flask(__name__)
sio = SocketIO(app, cors_allowed_origins="*")

image_subscriber = None
br = None

topics = {
    "front_cam": "/nautilus/nautilus/camera1/nautilus_cam/compressed",
    "down_cam": "/nautilus/nautilus/camera2/nautilus_cam/compressed",
    "img_sub": "/image/distribute"
}
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

    if id == 'front_cam' or id == 'down_cam':
        buffer = np.fromstring(data.data, np.uint8)
    else:
        image = br.imgmsg_to_cv2(data)
        retval, buffer = cv2.imencode('.png', image)
    img = base64.b64encode(buffer)
    sio.emit("Image Display", {'image': img, 'id': id}, broadcast = True)

@sio.on("Get IDs")
def send_ids():
    """
    Sends topics map to client

    This method sends all topic ids to the client as a JSON object stored as
    'ids'. THis method gets called by "Get IDs" and emits "IDs"
    back to the client.

    Parameters
    -------
    data : None

    Returns
    -------
    None
    """
    global topics
    print("sending list of IDs")
    ids = topics.keys()
    sio.emit("IDs", {'ids':ids}, broadcast = True)


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

        image_subscriber = rospy.Subscriber(topics['img_sub'], Image, send_image, 'img_sub')
        front_cam_subscriber = rospy.Subscriber(topics['front_cam'], CompressedImage, send_image, 'front_cam')
        downward_cam_subscriber = rospy.Subscriber(topics['down_cam'], CompressedImage, send_image, 'down_cam')

        br = CvBridge()
        sio.run(app, host=HOST_IP, port=HOST_PORT)
    except rospy.ROSInterruptException: pass
