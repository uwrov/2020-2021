#!/usr/bin/env python3
import rospy
from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit
from sensor_msgs.msg import Image, CompressedImage
import threading
from geometry_msgs.msg import Wrench
import image_server
import Move_Server

from engineio.payload import Payload
Payload.max_decode_packets = 50


HOST_IP = "localhost"
# HOST_IP = "0.0.0.0"
HOST_PORT = "4040"

app = Flask(__name__)
sio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

@sio.on("Get IDs")
def send_image_id():
    image_server.send_ids(sio)

@sio.on("Send State")
def send_move_state(data):
    Move_Server.update_state(data,sio)

if __name__ == '__main__':
    """ Sets up rospy and starts servers """
    try:

        print("main server is running")
        rospy.init_node('surface', anonymous=True)

        image_subscriber = rospy.Subscriber(image_server.topics['img_sub'], CompressedImage, image_server.send_image, ('img_sub', sio))
        front_cam_subscriber = rospy.Subscriber(image_server.topics['front_cam'], CompressedImage, image_server.send_image, ('front_cam', sio))
        downward_cam_subscriber = rospy.Subscriber(image_server.topics['down_cam'], CompressedImage, image_server.send_image, ('down_cam', sio))

        velocity_publisher = rospy.Publisher('/nautilus/thruster_manager/input', Wrench, queue_size=10)
        threading.Thread(target=Move_Server.publish, args=(velocity_publisher,)).start()

        sio.run(app, host=HOST_IP, port=HOST_PORT)
    except rospy.ROSInterruptException: pass
