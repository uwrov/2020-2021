#!/usr/bin/env python3
from geometry_msgs.msg import Wrench
from std_msgs.msg import Int16
import time

current = None
msg = Wrench()
channel = Int16()
channel.data = 1

def update_state(state, sio, channel_publisher):
    """
    Updates State based on new contoller input

    Receives movement information from controller as a JSON object.
    The movement info is then converted into a Wrench object

    Parameters
    -------
    state : JSON/Dictionary
        stores the movement of the controller in terms of linear components and
        anglular components.
        state = {lin_x: 10, lin_y: 0, lin_z: 0, ang_x: 0, ang_y: 0, ang_z: 3,
                 a: true, b: false, x: false, y: false}

    Returns
    -------
    None
    """
    global msg, current, channel, channel_publisher
    if (current is None or state != current):
        if (state["ang_x"] != 0 or state["ang_y"] != 0 or state["ang_z"] != 0):
            state["lin_x"] = 0
            state["lin_y"] = 0
            state["lin_z"] = 0

        msg.force.x = state["lin_x"]
        msg.force.y = state["lin_y"]
        msg.force.z = state["lin_z"]
        msg.torque.x = state["ang_x"]
        msg.torque.y = state["ang_y"]
        msg.torque.z = state["ang_z"]

        if (state["a"] == 1):
            channel.data = 0 # usb cam
        elif (state["b"] == 1):
            channel.data = 1 # picam a
        elif (state["x"] == 1):
            channel.data = 3 # picam b
        elif (state["y"] == 1):
            channel.data = 2 # picam c

        channel_publisher.publish(channel)
        current = state


def publish(velocity_publisher):
    """
    Publishes controller input to rospy

    publishes the wrench object, which stores the contoller input, to rospy
    every 20 milliseconds while the server is running

    Parameters
    -------
    buffer : Tuple
        takes in an arbitary tuple because the
        ```
        _thread.start_new_thread(function, tuple)
        ```
        method needs requires a tuple as the second argument tuple to call
        the function.
        This publish() function, does nothing with this value.

    Returns
    -------
    None
    """
    print('publishing')
    while True:
        velocity_publisher.publish(msg)
        # channel_publisher.publish(channel)
        time.sleep(.05)
