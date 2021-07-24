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

channel_publisher = None
@sio.on("Get IDs")
def send_image_id():
    image_server.send_ids(sio)

@sio.on("Send State")
def send_move_state(data):
    Move_Server.update_state(data,sio, channel_publisher)

if __name__ == '__main__':
    """ Sets up rospy and starts servers """
    try:
        print("main server is running")

        rospy.init_node('surface', anonymous=True) # get rid of anonymous?

        image_subscriber = rospy.Subscriber(image_server.topics['img_sub'], CompressedImage, image_server.send_image, ('img_sub', sio))
        camera_subscriber = rospy.Subscriber(image_server.topics['camera_stream'], CompressedImage, image_server.send_image, ('camera_stream', sio))

        velocity_publisher = rospy.Publisher('/nautilus/motors/commands', Wrench, queue_size=10)
        channel_publisher = rospy.Publisher('/nautilus/cameras/switch', Int16, queue_size=1)
        threading.Thread(target=Move_Server.publish, args=(velocity_publisher,)).start()

        sio.run(app, host=HOST_IP, port=HOST_PORT)
    except rospy.ROSInterruptException: pass
