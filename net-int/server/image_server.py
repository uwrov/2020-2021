#!/usr/bin/env python3
from std_msgs.msg import Int16

id = None
data = None
channel = Int16()
channel.data = 1

topics = {
    "camera_stream": "/nautilus/camera/stream",
    "img_sub": "/image/distribute"
}


def send_image(data, tup):
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
    id, sio = tup
    print(sio)
    sio.emit("Image Display", {'image': data.data, 'id': id}, broadcast = True)


def send_ids(sio):
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
    sio.emit("IDs", {'ids':ids}, broadcast=True)

def set_camera(state, channel_publisher):
    if (state["a"] == 1):
        channel.data = 0 # usb cam
    elif (state["b"] == 1):
        channel.data = 1 # picam a
    elif (state["x"] == 1):
        channel.data = 3 # picam b
    elif (state["y"] == 1):
        channel.data = 2 # picam c

    channel_publisher.publish(channel)
