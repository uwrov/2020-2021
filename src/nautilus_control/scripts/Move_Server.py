#!/usr/bin/env python3
import json
import rospy
from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit
from geometry_msgs.msg import Wrench
import _thread
import time


HOST_IP = "0.0.0.0"
HOST_PORT = "4041"

app = Flask(__name__)
sio = SocketIO(app, cors_allowed_origins="*")

# rate = None
velocity_publisher = None
current = None
msg = Wrench()


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
        state = {lin_x: 10, lin_y: 0, lin_z: 0, ang_x: 0, ang_y: 0, ang_z: 3}

    Returns
    -------
    None
    """
    global msg, current
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


# def move_init():
#     """ Sets up rospy and inital publisher thread """
#     print('move server is running')
#     try:
#         rospy.init_node('move_server', anonymous=False)
#         velocity_publisher = rospy.Publisher('/nautilus/thruster_manager/input', Wrench, queue_size=10)
#         _thread.start_new_thread(publish, (0,))
#     except rospy.ROSInterruptException: pass



if __name__ == '__main__':
    """ Sets up rospy, inital publisher thread, and starts server """
    try:
        rospy.init_node('move_server', anonymous=False)
        velocity_publisher = rospy.Publisher('/nautilus/thruster_manager/input', Wrench, queue_size=10)
        # rate = rospy.Rate(.10)
        _thread.start_new_thread(publish, (0,))
        sio.run(app, host=HOST_IP, port=HOST_PORT)
    except rospy.ROSInterruptException: pass
