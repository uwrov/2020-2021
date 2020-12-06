#!/usr/bin/env python3
import json
import rospy
import socketio

from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

HOST_IP = "localhost"
HOST_PORT = "4040"

image_publisher = None;

bridge = CvBridge()

sio = socketio.Client()

def send_image_through_ROS():
	img = cv2.imread('smile.png')
	message = bridge.cv2_to_imgmsg(img, encoding="bgr8")
	image_publisher.publish(message)


if __name__ == '__main__':
	""" Sets up rospy and starts server """
	try:
		rospy.init_node('image_boi', anonymous=True)
		image_publisher = rospy.Publisher('/images/distribute', Image, queue_size=10)
		rate = rospy.Rate(10)
		sio.connect('http://',HOST_IP,":",HOST_PORT)
	except rospy.ROSInterruptException: pass
	
	send_image_through_ROS()
