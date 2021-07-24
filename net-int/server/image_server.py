#!/usr/bin/env python3

topics = {
    "camera_stream": "/nautilus/camera/stream",
    "img_sub": "/image/distribute"
}
data = None
id = None

# def send_image(data, id, sio):
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
