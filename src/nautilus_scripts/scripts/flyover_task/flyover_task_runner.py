#!/usr/bin/env python3
"""
ROS driver for the flyover task
"""

import cv2
import rospy
import numpy as np
import threading
import time
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
    course_adjustment = threading.Thread(target=course_adjustment_thread, args=())
    data_collection = threading.Thread(target=data_collection_thread, args=())
    course_adjustment.start()
    data_collection.start()
    while not flying_over:
        rospy.sleep()
    course_adjustment.join()
    data_collection.join()
    
    result = generate_grid(data)
    print("saving result")
    cv2.imwrite(output_path, result)

def update_frame(msg):
    global data
    global flying_over
    global current_frame

    # get current frame
    np_arr = np.fromstring(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    current_frame = image_np.copy()

    if flying_over:
        if (len(data) == 9):
            if (len(data[8]) == 3):
                    if (data[8][2] != None):
                        flying_over = False

def course_adjustment_thread():
    global current_frame
    global flying_over
    if flying_over:
        instructions = course_correct(current_frame)
        adjust_course(instructions)
        time.sleep(3)
    else:
        return

def data_collection_thread():
    global current_frame
    global flying_over
    if flying_over:
        if frame_contains_row(current_frame):
            data.append(snip_row(current_frame))
        time.sleep(3)
    else:
        return

def shutdown_fn():
    print("shutting down")

if __name__ == '__main__':
    main()
