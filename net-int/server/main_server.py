#!/usr/bin/env python3
import rospy
from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit
from sensor_msgs.msg import Image, CompressedImage
import _thread
import time

HOST_IP = "localhost"
# HOST_IP = "0.0.0.0"
HOST_PORT = "4040"

app = Flask(__name__)
sio = SocketIO(app, cors_allowed_origins="*")


@sio.on("Get IDs")
def send_image_id():
    send_ids()

from image_server import *
# from Move_Server import *


if __name__ == '__main__':
    """ Sets up rospy and starts servers """
    try:
        print("main server is running")
        rospy.init_node('surface', anonymous=True)

        image_subscriber = rospy.Subscriber(topics['img_sub'], CompressedImage, send_image, 'img_sub')
        front_cam_subscriber = rospy.Subscriber(topics['front_cam'], CompressedImage, send_image, 'front_cam')
        downward_cam_subscriber = rospy.Subscriber(topics['down_cam'], CompressedImage, send_image, 'down_cam')

        # velocity_publisher = rospy.Publisher('/nautilus/thruster_manager/input', Wrench, queue_size=10)
        # _thread.start_new_thread(publish, (velocity_publisher,))

        sio.run(app, host=HOST_IP, port=HOST_PORT)
    except rospy.ROSInterruptException: pass


        # image_server.start_server()
        # time.sleep(5)
        # Move_Server.start_server()
        # time.sleep(5)

        # while True:
        #     choice = input("press q to exit: ")
        #     print()
        #     if choice == 'q':
        #         print('exiting')
        #         break
        # sio.run(app, host=HOST_IP, port=HOST_PORT)
