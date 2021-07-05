#!/usr/bin/env python3
"""
ROS driver for the flyover task
"""

import cv2
import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage

# variables
current_frame = None
instructions = None
data = []

flying_over = True
result = None

# paths
cam1 = '/nautilus/nautilus/camera1/nautilus_cam/compressed'
output_path = '/home/uwrov/Desktop/flyover_output.png'

def main():
    r = rospy.Rate(5)
    rospy.Subscriber(cam1, CompressedImage, update_frame)
    while not flying_over:
        rospy.sleep()
    result = generate_grid(data)
    print("saving result")
    cv2.imwrite(output_path, result)

def update_frame():
    global data
    if flying_over:
        # course adjustment
        instructions = course_correct(current_frame)
        adjust_course(instructions)
        
        # data collection
        if frame_contains_row(current_frame):
            data = snip_row(current_frame)

        # update flying over?

def shutdown_fn():
    print("shutting down")

if __name__ == '__main__':
    main()
