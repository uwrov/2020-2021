#!/usr/bin/env python3
import json
import rospy
import socketio
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import base64

HOST_IP = "localhost"
HOST_PORT = "4040"

image_publisher = None;
bridge = CvBridge()

sio = socketio.Client()

def send_image_through_ROS():
    img = cv2.imread('smile.png')
    message = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    image_publisher.publish(message)
    print("image sent")

# @sio.on("Image Display")
# def get_image(data):
#     print("image recieved")
#     with open("imageToSave.png", "wb") as fh:
#         fh.write(base64.decodebytes(data['image']))
#     img = cv2.imread('imageToSave.png')
#     cv2.imshow("Image",img)
#     cv2.waitKey(0)


if __name__ == '__main__':
    """ Sets up rospy and starts server """
    try:
        image_publisher = rospy.Publisher('/image/distribute', Image, queue_size=10)
        rospy.init_node('wheely_boi', anonymous=True)
        #rate = rospy.Rate(10)
        sio.connect('http://'+HOST_IP+":"+HOST_PORT)
    except rospy.ROSInterruptException: pass

    send_image_through_ROS()
