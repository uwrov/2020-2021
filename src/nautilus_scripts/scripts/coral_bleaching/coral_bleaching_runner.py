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
from coral_bleaching.match_images import match_images

# static variables
cam = '/nautilus/nautilus/camera1/nautilus_cam/compressed'
button = '/buttonPress'
output = match_images()

# paths - need adjustment
original_image_path = 'path_to_image'
output_path = 'path_to_output'

def main():
    rospy.init_node('coral_bleaching_runner')
    rospy.Subscriber(cam, CompressedImage, update_frame)
    old_picture = cv2.imread(original_image_path)
    rospy.Subscriber('button', snapshot_fn)
    
    rospy.on_shutdown(shutdown_fn)
    while not rospy.is_shutdown():
        if output.is_finished():
            export_output(output.get_output())
        rospy.sleep(.5)


def snapshot_fn():
    cap = cameras.get_frame(1)
    if cap is not None:
        # process code goes here
    return


def update_frame(msg):
    # update frame code here
    np_arr = np.fromstring(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    cameras.update_frame(image_np, 1)


def export_output(output):
    cv2.imwrite(output_path, output.get_output())
    print("done!")
    rospy.signal_shutdown("finished executing")


def shutdown_fn():
    print("shutting down")


if __name__ == '__main__':
    main()