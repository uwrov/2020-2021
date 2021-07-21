#!/usr/bin/env python3
# import rospy
# from flask import Flask, render_template
# from flask_socketio import SocketIO, send, emit
# from sensor_msgs.msg import Image, CompressedImage
from main_server import sio

# HOST_IP = "localhost"
# # HOST_IP = "0.0.0.0"
# HOST_PORT = "4040"
#
# app = Flask(__name__)
# sio = SocketIO(app, cors_allowed_origins="*")

topics = {
    "front_cam": "/nautilus/nautilus/camera1/nautilus_cam/compressed",
    "down_cam": "/nautilus/nautilus/camera2/nautilus_cam/compressed",
    "img_sub": "/image/distribute"
}

def send_image(data, id):
    """
    Sends image to client

    Receives movement information from ROS as a image message.
    The image is then converted into a cv2 image and then encoded in base 64 so
    it can be sent as a JSON object to the client via socket.io

    Parameters
    -------
    data : ROS image
        stores image from ROS
    id: a string representing id topic

    Returns
    -------
    None
    """
    sio.emit("Image Display", {'image': data.data, 'id': id}, broadcast = True)


def send_ids():
    """
    Sends topics map to client

    This method sends all topic ids to the client as a JSON object stored as
    'ids'. THis method gets called by "Get IDs" and emits "IDs"
    back to the client.

    Parameters
    -------
    None

    Returns
    -------
    None
    """
    global topics
    print("sending list of IDs")
    ids = list(topics.keys())
    sio.emit('hello', 'hello')
    sio.emit("IDs", {'ids':ids}, broadcast=True)


# def image_init(buffer):
#     """ Sets up rospy and subscibers """
#     try:
#         print("image server is running")
#
#         image_subscriber = rospy.Subscriber(topics['img_sub'], CompressedImage, send_image, 'img_sub')
#         front_cam_subscriber = rospy.Subscriber(topics['front_cam'], CompressedImage, send_image, 'front_cam')
#         downward_cam_subscriber = rospy.Subscriber(topics['down_cam'], CompressedImage, send_image, 'down_cam')
#
#         try:
#             sio.run(app, host=HOST_IP, port=HOST_PORT)
#         except sio.error as socketerror:
#             print("Error: ", socketerror)
#
#     except rospy.ROSInterruptException: pass
#
# def start_server():
#     _thread.start_new_thread(image_init, (0,))
