#!/user/bin/env python3
"""
ROS driver for the subway car photomosaic task,
orignal code authored by
Peyton Lee, Margot Adam, Cindy Zou and Jonathan Wong
"""

import cv2
import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from coral_bleaching.match_images import match_images

#static variables
cam1 = '/nautilus/nautilus/camera1/nautilus_cam/compressed'
original_image_path = 'path_to_image'

def main():
    rospy.init_node('coral_bleaching_runner')
    rospy.Subscriber(cam1, CompressedImage, update_frame)
    old_picture = cv2.imread(original_image_path)
    rospy.Subscriber('button', snapshot_fn)
    
    # ask about rospy.spin()
    rospy.on_shutdown(shutdown_fn)

    while not rospy.is_shutdown():
        if output.is_finished():
            export_output(output.get_output())
        rospy.sleep(.5)


def snapshot_fn():


def update_frame():

def shutdown_fn():
    print("shutting down")

if __name__ == '__main__':
    main()