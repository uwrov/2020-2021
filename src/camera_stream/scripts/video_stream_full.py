import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2

# https://qengineering.eu/install-opencv-4.4-on-raspberry-pi-4.html

# sudo apt-get update
# sudo apt-get install cmake gfortran libjpeg-dev libtiff-dev libgif-dev libavcodec-dev libavformat-dev libswscale-dev libgtk2.0-dev libcanberra-gtk* libxvidcore-dev libx264-dev libgtk-3-dev libtbb2 libtbb-dev libdc1394-22-dev libv4l-dev libopenblas-dev libatlas-base-dev libblas-dev libjasper-dev liblapack-dev libhdf5-dev gcc-arm* protobuf-compiler libgstreamer1.0-dev libilmbase-dev libopenexr-dev

# remember to add ROS_MASTER_URI
#                 ROS_IP

if __name__ == '__main__':
    src = 'http://192.168.0.3:8080/video/mjpeg'
    stream = cv2.VideoCapture(src)
    
    image_pub = rospy.Publisher('out/img', Image, queue_size=1)
    rospy.init_node('test_video_stream')
    rate = rospy.Rate(100)

    msg = Image()
    while not rospy.is_shutdown():
        ret, frame = stream.read()
        msg.height=frame.shape[0]
        msg.width=frame.shape[1]
        msg.header.stamp = rospy.Time.now()
        msg.data = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.uint8).flatten().tolist()
        # msg.data = [0]*(640*480)
        image_pub.publish(msg)
        rate.sleep()