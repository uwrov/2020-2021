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
from coral_bleaching.match_images import MatchImages

# static variables
cam = '/nautilus/nautilus/camera1/nautilus_cam/compressed'
button = '/buttonPress'
output = match_images()
current_frame = None

# paths - need adjustment
original_image_path = 'path_to_image'
output_path = 'path_to_output'

def main():
    rospy.init_node('coral_bleaching_runner')
    rospy.Subscriber(cam, CompressedImage, update_frame)
    old_picture = cv2.imread(original_image_path)
    rospy.Subscriber('button', String, snapshot_fn)
    
    rospy.on_shutdown(shutdown_fn)
    rospy.spin()
    

def update_frame(msg):
    np_arr = np.fromstring(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    current_frame = image_np.copy()

def snapshot_fn(msg):
    cap = cameras.get_frame(1)
    if cap is not None:
        # process code goes here - send frame (current_frame) to process function
        # output to output_path then shut down -> rospy.signal_shutdown("finished executing")
    return


def shutdown_fn():
    print("shutting down")


if __name__ == '__main__':
    main()