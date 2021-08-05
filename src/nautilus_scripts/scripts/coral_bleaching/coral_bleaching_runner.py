#!/usr/bin/env python3
"""
ROS driver for the coral bleaching task,
orignal code authored by
Peyton Lee, Margot Adam, Cindy Zou and Jonathan Wong
"""

import cv2
import rospy
import numpy as np
import coral_bleaching.coral as coral_bleaching
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Empty

# variables
cam = '/nautilus/cameras/stream'
button = '/nautilus/controls/signal'
current_frame = None
old_picture = None

# paths
original_image_path = '/home/uwrov/original.png'
old_coral_output_path = '/home/uwrov/old_coral_ouptut.png'
new_coral_ouptut_path = '/home/uwrov/new_coral_ouptut.png'

# test paths
captured_image_path = '/home/uwrov/capture.png'

def main():
    # test
    # global capture
    # capture = cv2.imread(captured_image_path)

    # non-test
    global old_picture
    rospy.init_node('coral_bleaching_runner')
    rospy.Subscriber(cam, CompressedImage, update_frame)
    old_picture = cv2.imread(original_image_path)
    rospy.Subscriber(button, Empty, snapshot_fn)
    rospy.on_shutdown(shutdown_fn)
    rospy.spin()

def update_frame(msg):
    global current_frame
    np_arr = np.fromstring(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    current_frame = image_np.copy()

def snapshot_fn(msg):
    global old_picture
    global current_frame
    cap = current_frame
    if cap is not None:
        # oldCoral_rect, newCoral_rect = coral_bleaching.run_task(old_picture, capture)
        oldCoral_rect, newCoral_rect = coral_bleaching.run_task(old_picture, current_frame)
        cv2.imwrite(new_coral_ouptut_path, newCoral_rect)
        cv2.imwrite(old_coral_output_path, oldCoral_rect)
        rospy.signal_shutdown("finished executing")
    else: 
        print("nothing")
    return

def shutdown_fn():
    print("shutting down")

if __name__ == '__main__':
    main()
