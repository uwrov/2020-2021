#!/usr/bin/env python3
import json
import rospy
from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit
from geometry_msgs.msg import Wrench
from std_msgs.msg import Int16
import _thread
import time

HOST_IP = "0.0.0.0"
HOST_PORT = "4041"

app = Flask(__name__)
sio = SocketIO(app, cors_allowed_origins="*")

velocity_publisher = None
# rate = None
channel_publisher = None
current = None
msg = Wrench()
channel = Int16()
channel.data = 1


def update_state(state):
    """
    Updates State based on new contoller input

    Receives movement information from controller as a JSON object.
    The movement info is then converted into a Wrench object

    Parameters
    -------
    state : JSON/Dictionary
        stores the movement of the controller in terms of linear components and
        anglular components.
        state = {lin_x: 10, lin_y: 0, lin_z: 0, ang_x: 0, ang_y: 0, ang_z: 3, a: true, b: false, x: false, y: false}

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

        print(state)

        if (state["a"] == 1):
            channel.data = 0 # usb cam
            channel_publisher.publish(channel)
        elif (state["b"] == 1):
            channel.data = 1 # picam a
            channel_publisher.publish(channel)
        elif (state["x"] == 1):
            channel.data = 3 # picam b
            channel_publisher.publish(channel)
        elif (state["y"] == 1):
            channel.data = 2 # picam c
            channel_publisher.publish(channel)
        
        current = state


@sio.on("Send State")
def send_state(state):
    """
    Creates a new thread to update state

    Receives movement information from controller as a JSON object.
    This method then creates a new thread and calls the update_state()
    to update the state with the new movement input.

    Parameters
    -------
    state : JSON/Dictionary
        stores the movement of the controller in terms of linear components and
        anglular components.
        state = {lin_x: 10, lin_y: 0, lin_z: 0, ang_x: 0, ang_y: 0, ang_z: 3}

    Returns
    -------
    None
    """
    _thread.start_new_thread(update_state, (state, ))


def publish(buffer):
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
    while True:
        # rospy.loginfo("Sending Command v:" + str(current))
        velocity_publisher.publish(msg)
        time.sleep(.05)
        # rate.sleep()


if __name__ == '__main__':
    """ Sets up rospy, inital publisher thread, and starts server """
    try:
        rospy.init_node('move_server')
        velocity_publisher = rospy.Publisher('/nautilus/motors/commands', Wrench, queue_size=10)
        channel_publisher = rospy.Publisher('/nautilus/cameras/switch', Int16, queue_size=1)
        # rate = rospy.Rate(.10)
        _thread.start_new_thread(publish, (0,))
        sio.run(app, host=HOST_IP, port=HOST_PORT)
    except rospy.ROSInterruptException: pass
