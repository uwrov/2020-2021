#!/usr/bin/env python3 
import rospy
import time
import picamera
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import threading
import multiprocessing


class CamBuffer:
    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height
        self.data = None


    def write(self, s):
        self.data = s


    def get_data(self):
        return self.data


def align_down(size, align):
    return (size & ~((align)-1))


def align_up(size, align):
    return align_down(size + align - 1, align)


def picam_init(camera):
    global cam_stream
    global pidims
    global fr
    
    cam_stream = CamBuffer(pidims[0], pidims[1])
    camera.resolution = (pidims[0], pidims[1])
    camera.framerate = fr
    
    camera.start_recording(cam_stream, format='mjpeg')


def usbcam_init(usb_fr, width, height, stream):    
    stream.set(cv2.CAP_PROP_FPS, usb_fr)
    stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)


def camPickerCallback(msg, flagArr):
    flagArr[0] = msg.data == 'cam1'


if __name__ == '__main__':
    src = 0
    stream = cv2.VideoCapture(src)

    global cam_stream
    global pidims
    global usbdims
    camera = picamera.PiCamera()

    pidims = [480, 360]
    usbdims = (480, 360)

    fr = 20
    
    def shutdown_fn():
        global cam_stream
        camera.close()
        stream.release()
        print('shutting down')

    rospy.init_node('compress_stream')
    rospy.on_shutdown(shutdown_fn)
    topic_name = '/nautilus/cameras/switch'
    
    picam_selected = [True]

    front_cam = rospy.Publisher('/nautilus/cameras/stream', CompressedImage, queue_size=0)
    rospy.Subscriber(topic_name, String, camPickerCallback, callback_args=(picam_selected))

    msg = CompressedImage()
    msg.format = 'jpeg'

    picam_init(camera)
    usbcam_init(fr, usbdims[0], usbdims[1], stream)

    rate = rospy.Rate(fr)

    while not rospy.is_shutdown():
        if picam_selected[0]:
            msg.data = cam_stream.get_data()
            if msg.data is not None:
                front_cam.publish(msg)
            rate.sleep()
        else:
            ret, frame = stream.read()
            msg.header.stamp = rospy.Time.now()
            msg.data = np.array(cv2.imencode('.jpeg', frame)[1]).tostring()
            front_cam.publish(msg)
            rate.sleep()
