#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import MultiArrayLayout, UInt8MultiArray, MultiArrayDimension
import numpy as np
import cv2 
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

camera = arducam.mipi_camera()
camera.init_camera()

def align_down(size, align):
    return (size & ~((align)-1))

def align_up(size, align):
    return align_down(size + align - 1, align)

def main():
    global camera
    rospy.init_node('camera_streamer')
    rospy.on_shutdown(shutdown_fn)

    front_cam = rospy.Publisher('/nautilus/nautilus/camera1/nautlius_cam/yuv', UInt8MultiArray, queue_size=1)
    down_cam  = rospy.Publisher('/nautilus/nautilus/camera2/nautlius_cam/yuv', UInt8MultiArray, queue_size=1)

    fmt = camera.set_resolution(1280, 480)
    print("Current resolution is {}".format(fmt))

    msg = UInt8MultiArray()
    msg.layout = MultiArrayLayout()
    msg.layout.data_offset = 0
    msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
    msg.layout.dim[0].label = 'height'
    msg.layout.dim[0].size  = fmt[1]
    msg.layout.dim[1].label = 'width'
    msg.layout.dim[1].size  = fmt[0]//2

    height = int(align_up(fmt[1], 16))
    width = int(align_up(fmt[0], 32))

    while not rospy.is_shutdown():
        frame = camera.capture(encoding = 'i420')
        img = frame.as_array
        img = img.reshape(int(height * 1.5), width)

        # separate the two camera streams
        i1, i2 = np.hsplit(img, 2)

        # publish our messages
        msg.data = i1.flatten().tolist()
        front_cam.publish(msg)
        msg.data = i2.flatten().tolist()
        down_cam.publish(msg)

        del frame

def shutdown_fn():
    global camera
    camera.close_camera()
    print('shutting down')

if __name__ == "__main__":
    main()
