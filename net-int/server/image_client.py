#!/usr/bin/env python3
import json
import rospy
import socketio
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge, CvBridgeError
import base64

HOST_IP = "localhost"
HOST_PORT = "4040"

image_publisher = None
bridge = CvBridge()

sio = socketio.Client()

def send_image_through_ROS():
    """
    Publishes an image through ROS

    Converts a png to a compressedImage and then publishes it through ROS
    through the '/image/distribute' topic

    Parameters
    -------
    None

    Returns
    -------
    None
    """
    img = cv2.imread('smile.png')
    message = bridge.cv2_to_compressed_imgmsg(img)
    image_publisher.publish(message)
    print("image sent")

@sio.on('Scripts List')
def print_scripts(data):
    print(data['scripts'])


if __name__ == '__main__':
    """ Sets up rospy and starts server """
    try:
        image_publisher = rospy.Publisher('/image/distribute', CompressedImage, queue_size=10)
        rospy.init_node('wheely_boi', anonymous=True)
        sio.connect('http://'+HOST_IP+":"+HOST_PORT)
    except rospy.ROSInterruptException: pass

    send_image_through_ROS()
    sio.emit('Get Scripts')
