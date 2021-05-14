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
dims = (640, 480)
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
    while not rospy.is_shutdown():
        camera.capture(cam_stream, 'jpeg', use_video_port=True)
        msg.data = list(cam_stream.get_data())
        front_cam.publish(msg)        
        rate.sleep()

def shutdown_fn():
    global cam_stream
    global camera
    print('shutting down')

if __name__ == '__main__':
    main()
