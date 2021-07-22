#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from MultiCam.MultiCamManager import MultiCamManager
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int16

if __name__ == '__main__':
    cam_manager = MultiCamManager(nPiCam=3, useUsbCam=True)
    fr = 20

    pub_topic = '/nautilus/cameras/stream'
    sub_topic = '/nautilus/cameras/switch'

    rospy.init_node('cam_streamer')
    rospy.on_shutdown(cam_manager.close)
    rospy.Subscriber(sub_topic, Int16, cam_manager.cam_switch_callback)

    pub = rospy.Publisher(pub_topic, CompressedImage, queue_size=10)
    msg = CompressedImage()
    msg.format = 'jpeg'

    rate = rospy.Rate(fr)

    print("Streaming on", pub_topic)

    while not rospy.is_shutdown():
        _, data = cam_manager.read_current_channel()
        msg.data = np.array(cv2.imencode('.jpeg', data)[1]).tostring()
        pub.publish(msg)
        rate.sleep()