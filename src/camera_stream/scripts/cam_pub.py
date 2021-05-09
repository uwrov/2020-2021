#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import MultiArrayLayout, UInt8MultiArray
import numpy as np
import cv2 
from picamera import PiCamera
from time import sleep
import time
import io
import arducam_mipicamera as arducam

# One-time steps to use the publisher: 
#   sudo apt-get update
#   sudo apt-get install cmake gfortran libjpeg-dev libtiff-dev libgif-dev libavcodec-dev libavformat-dev libswscale-dev libgtk2.0-dev libcanberra-gtk* libxvidcore-dev libx264-dev libgtk-3-dev libtbb2 libtbb-dev libdc1394-22-dev libv4l-dev libopenblas-dev libatlas-base-dev libblas-dev libjasper-dev liblapack-dev libhdf5-dev gcc-arm* protobuf-compiler libgstreamer1.0-dev libilmbase-dev libopenexr-dev
#
# Every-time steps to use the publisher: 
#   In the terminal: export ROS_MASTER_URI=http://[VM IP]:11311
#   In the terminal: export ROS_IP=[Pi IP]
#   Run file (cd 2020-2021/src/camera_stream/scripts/, then python3 cam_pub.py)
#
# Future updates: 
#   Need to edit source for stream2 to take input from second camera (see line 30)

def streaming(msg, rate, stream, image_pub): 
    while not rospy.is_shutdown():
        ret, frame = stream.read()
        msg.header.stamp = rospy.Time.now()
        msg.format = 'jpeg'
        msg.data = np.array(cv2.imencode('.jpeg', frame)[1]).tostring()
        image_pub.publish(msg)
        rate.sleep()

def streaming2(msg, rate, stream, camera, image_pub):
    camera.capture(stream, format='jpeg')
    while not rospy.is_shutdown():        
        msg.data = stream.getValue()
        image_pub.publish(msg)
        rate.sleep()

def align_down(size, align):
    return (size & ~((align)-1))

def align_up(size, align):
    return align_down(size + align - 1, align)

if __name__ == "__main__":
    try:
        camera = arducam.mipi_camera()
        print("Open camera...")
        camera.init_camera()
        print("Setting the resolution...")
        fmt = camera.set_resolution(1280, 480)
        print("Current resolution is {}".format(fmt))

        image_pub = rospy.Publisher('out/img/compressed', UInt8MultiArray, queue_size=1)
        rospy.init_node('compress_stream')
        rate = rospy.Rate(100)

        msg = UInt8MultiArray()
        msg.layout = MultiArrayLayout()
        msg.layout.data_offset = 0
        while cv2.waitKey(10) != 27:
            frame = camera.capture(encoding = 'i420')
            height = int(align_up(fmt[1], 16))
            width = int(align_up(fmt[0], 32))
            image = frame.as_array

            msg.data = image.tolist()
            
            image_pub.publish(msg)

        # Release memory
        del frame
        print("Close camera...")
        camera.close_camera()
    except Exception as e:
        print(e) 