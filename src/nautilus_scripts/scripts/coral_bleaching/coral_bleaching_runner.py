#!/user/bin/env python3
"""
ROS driver for the coral bleaching task,
orignal code authored by
Peyton Lee, Margot Adam, Cindy Zou and Jonathan Wong
"""

import cv2
import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import coral_bleaching

# variables
cam = '/nautilus/nautilus/camera1/nautilus_cam/compressed'
button = '/buttonPress'
current_frame = None
old_picture = None

# paths - need adjustment
original_image_path = '~/original.png'
old_coral_output_path = '~/old_coral_ouptut.png'
new_coral_ouptut_path = '~/new_coral_ouptut.png'

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
        oldCoral_rect, newCoral_rect = coral_bleaching.run_task(old_picture, current_frame)
        cv2.imwrite(new_coral_ouptut_path, newCoral_rect)
        cv2.imwrite(old_coral_output_path, oldCoral_rect)
        rospy.signal_shutdown("finished executing")
    return

def shutdown_fn():
    print("shutting down")

if __name__ == '__main__':
    main()