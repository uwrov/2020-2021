#!/usr/bin/env python3 
import rospy
import time
import picamera
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
import threading

################################################
# Pi Camera Code

class CamBuffer:
    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height
        self.data = None

    def write(self, s):
        self.data = s

    def get_data(self):
        return self.data


camera = picamera.PiCamera()
dims = (480, 360)
cam_stream = CamBuffer(dims[0], dims[1])
fr = 12


def align_down(size, align):
    return (size & ~((align)-1))

def align_up(size, align):
    return align_down(size + align - 1, align)

def pi_main():
    global cam_stream
    global camera
    global dims
    rospy.on_shutdown(shutdown_fn)

    front_cam = rospy.Publisher('/nautilus/nautilus/camera1/nautilus_cam/compressed', CompressedImage, queue_size=0)

    msg = CompressedImage()
    msg.format = 'jpeg'

    camera.resolution = (dims[0], dims[1])
    camera.framerate = fr

    rate = rospy.Rate(fr)
    print('starting stream')
    camera.start_recording(cam_stream, format='mjpeg')
    while not rospy.is_shutdown():
        msg.data = cam_stream.get_data()
        if msg.data is not None:
            front_cam.publish(msg)       
        rate.sleep()

def shutdown_fn():
    global cam_stream
    global camera
    camera.stop_recording()
    camera.close()
    print('shutting down')

################################################
# USB Camera Code 

# One-time steps to use the publisher: 
#   sudo apt-get update
#   sudo apt-get install cmake gfortran libjpeg-dev libtiff-dev libgif-dev libavcodec-dev libavformat-dev libswscale-dev libgtk2.0-dev libcanberra-gtk* libxvidcore-dev libx264-dev libgtk-3-dev libtbb2 libtbb-dev libdc1394-22-dev libv4l-dev libopenblas-dev libatlas-base-dev libblas-dev libjasper-dev liblapack-dev libhdf5-dev gcc-arm* protobuf-compiler libgstreamer1.0-dev libilmbase-dev libopenexr-dev
#
# Every-time steps to use the publisher: 
#   In the terminal: export ROS_MASTER_URI=http://[VM IP]:11311
#   In the terminal: export ROS_IP=[Pi IP]
#   Run file (cd 2020-2021/src/camera_stream/scripts/, then python3 cam_pub.py)

def usb_main():    
    image_pub = rospy.Publisher('nautilus/nautilus/camera2/nautilus_cam/compressed', CompressedImage, queue_size=1)
    r = rospy.Rate(12)
    msg = CompressedImage()

    while not rospy.is_shutdown():
        ret, frame = stream.retrieve()
        msg.header.stamp = rospy.Time.now()
        msg.format = 'jpeg'
        msg.data = np.array(cv2.imencode('.jpeg', frame)[1]).tostring()
        image_pub.publish(msg)
        r.sleep()

def get_cam_stream():
    while not rospy.is_shutdown():
        stream.grab()
        time.sleep(1/12)

################################################
# main

if __name__ == '__main__':
    rospy.init_node('compress_stream')

    global stream 
    src = 0 # 'http://[Pi IP]:8081/'
    stream = cv2.VideoCapture(src)
    stream.set(cv2.CAP_PROP_FPS, 12)
    stream.set(cv2.CAP_PROP_FRAME_WIDTH, dims[0])
    stream.set(cv2.CAP_PROP_FRAME_HEIGHT, dims[1])

    pi_thread = threading.Thread(target=pi_main)
    usb_thread = threading.Thread(target=usb_main)
    usb_input = threading.Thread(target=get_cam_stream)
    
    pi_thread.start()
    usb_input.start()
    usb_thread.start() 