#!/usr/bin/env python3 
import time
import picamera
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import CompressedImage

class CamBuffer:
    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height
        self.data = None

    def write(self, s):
        self.data = s

    def get_data(self):
        return self.data


cam_stream = CamBuffer()
camera = picamera.PiCamera()
dims = (720, 480)
fr = 24 


def align_down(size, align):
    return (size & ~((align)-1))

def align_up(size, align):
    return align_down(size + align - 1, align)

def main():
    global cam_stream
    global camera
    global dims
    rospy.init_node('camera_streamer')
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

if __name__ == '__main__':
    main()
