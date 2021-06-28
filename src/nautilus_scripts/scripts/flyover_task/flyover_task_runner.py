#!/usr/bin/env python3
"""
ROS driver for the flyover task
"""

import cv2
import rospy
import numpy as np
import flyover_task.flyover as flyover
import flyover_task.process_image as process
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

# variables
cam = '/nautilus/nautilus/camera1/nautilus_cam/compressed'
button = '/buttonPress'

def main():
    return

def shutdown_fn():
    print("shutting down")

if __name__ == '__main__':
    main()
