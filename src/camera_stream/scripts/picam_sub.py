#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage

cam1 = '/nautilus/nautilus/camera1/nautilus_cam/compressed'
cam2 = '/nautilus/nautilus/camera2/nautilus_cam/compressed'

def display(msg, window_name):
    buf = np.frombuffer(msg.data, dtype=np.ubyte)
    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    cv2.imshow(window_name, img)
    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('sub')
    rospy.Subscriber(cam1, CompressedImage, display, callback_args=('cam1'))
    #rospy.Subscriber(cam2, CompressedImage, display, callback_args=('cam2'))

    rospy.on_shutdown(lambda : print('shutting down'))

    rospy.spin()
