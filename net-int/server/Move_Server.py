#!/usr/bin/env python3
from geometry_msgs.msg import Wrench
import time

current = None
msg = Wrench()

def update_state(state, sio):
    """
    Updates State based on new contoller input
    Receives movement information from controller as a JSON object.
    The movement info is then converted into a Wrench object
    Parameters
    -------
    state : JSON/Dictionary
        stores the movement of the controller in terms of linear components and
        anglular components.
        state = {lin_x: 0, lin_y: 0, lin_z: 0, ang_x: 0, ang_y: 0, ang_z: 0,
                 a: true, b: false, x: false, y: false}
    Returns
    -------
    None
    """
    global msg, current, channel
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
        time.sleep(.05)
