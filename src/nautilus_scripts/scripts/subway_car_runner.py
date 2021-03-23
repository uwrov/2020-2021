#!/usr/bin/env python3
"""
ROS driver for the subway car photomosaic task,
original code authored by
Lauren Krieger and Bennedict Soesanto
lkrieg@uw.edu   /  ben2111@uw.edu 
"""

import cv2
import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from subway_car.Photomosaic import Photomosaic
from nautilus_utils.CamHolder import CamHolder

# Static Variables
cam1 = '/nautilus/nautilus/camera1/nautilus_cam/compressed'
cam2 = '/nautilus/nautilus/camera1/nautilus_cam/compressed'
# cam1 = 'out/img/compressed'
# cam2 = 'out/img/compressed2'
button = '/buttonPress'

# Mutable Variables
cameras = CamHolder()
output = Photomosaic()

def main():
    rospy.init_node('subway_car_runner')
    rospy.Subscriber(cam1, CompressedImage, update_frame, callback_args=(1))
    rospy.Subscriber(cam2, CompressedImage, update_frame, callback_args=(2))
    rospy.Subscriber(button, String, snapshot_fn)

    rospy.on_shutdown(shutdown_fn)

    while not rospy.is_shutdown():
        if output.is_finished():
            cv2.imwrite('/home/uwrov/Desktop/out.png', output.get_output())
            print("done!")
            rospy.signal_shutdown("finished executing")
        rospy.sleep(.5)

def snapshot_fn(msg):
    if msg.data not in ['o', 't']:
        return
    cap = cameras.get_frame(1) if msg.data[0] == 'o' else cameras.get_frame(2)
    if cap is not None:
        print('adding frame', output.imageIndex)
        cv2.imwrite('/home/uwrov/Desktop/out' + str(output.imageIndex) + '.png', cap)
        
        output.add(cap)
    return


def update_frame(msg, target_cam):
    np_arr = np.fromstring(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    # print(image_np)
    if target_cam == 1:
        cameras.update_frame(image_np, 1)
    else:
        cameras.update_frame(image_np, 2)


def shutdown_fn():
    print("shutting down")

if __name__ == '__main__':
    main()