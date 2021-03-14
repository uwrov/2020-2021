import rospy
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2 
import threading

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

if __name__ == '__main__':
    src = 'http://192.168.1.10:8081/'
    src2 = 'http://192.168.1.10:8081/' # change sources as necessary 
    stream = cv2.VideoCapture(src)
    stream2 = cv2.VideoCapture(src2)
    
    image_pub = rospy.Publisher('out/img/compressed', CompressedImage, queue_size=1)
    image_pub2 = rospy.Publisher('out/img/compressed2', CompressedImage, queue_size=1)
    rospy.init_node('compress_stream')
    rate = rospy.Rate(100)

    msg = CompressedImage()

    # Publishes src to 'out/img/compressed'
    x = threading.Thread(target = streaming, args = (msg, rate, stream, image_pub))

    # Publishes src2 to 'out/img/compressed2'
    y = threading.Thread(target = streaming, args = (msg, rate, stream2, image_pub2))

    x.start()
    y.start()