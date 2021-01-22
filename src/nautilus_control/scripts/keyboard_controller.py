#!/usr/bin/env python3

import rospy
import curses
from geometry_msgs.msg import Wrench, Vector3

# Grab input from user (nonblocking)
def getch_c(stdscr):
    # do not wait for input when calling getch
    stdscr.nodelay(1)

    # get keyboard input, returns -1 if none available
    c = stdscr.getch()
    if c != -1:
        return c


def move():
    rospy.init_node('nautilus_teleop', anonymous=False)

    wrench_publisher = rospy.Publisher('/nautilus/thruster_manager/input', Wrench, queue_size=10)
    rate = rospy.Rate(10)

    w = Wrench()
    w.force = Vector3(0,0,0)
    w.torque = Vector3(0,0,0)

    force_update = [ord('w'), ord('s'), ord('a'), ord('d'), ord('x'), ord('z'), ord('m')]
    torque_update = [ord('j'), ord('l'), ord('m')]
    while not rospy.is_shutdown():
        key = curses.wrapper(getch_c)
        if key in force_update:
            update_func(key, w.force, force_update)
        if key in torque_update:
            update_func(key, w.torque, torque_update)

        wrench_publisher.publish(w)
        rate.sleep()

def inc(val):
    return min(val+0.1, 1.0)

def dec(val):
    return max(val-0.1, -1.0)

# Returns a Vector3 representing the update to be done to w
def update_func(key, vec, cmd_list):
    if (key == cmd_list[0]):
        vec.x = inc(vec.x)
    elif (key == cmd_list[1]):
        vec.x = dec(vec.x)
    elif (key == cmd_list[2]):
        vec.y = inc(vec.y)
    elif (key == cmd_list[3]):
        vec.y = dec(vec.y)
    elif (key == cmd_list[4]):
        vec.z = inc(vec.z)
    elif (key == cmd_list[5]):
        vec.z = dec(vec.z)
    elif (key == cmd_list[6]):
        vec.x = 0
        vec.y = 0
        vec.z = 0


if __name__ == '__main__':
    # Print welcome screen
    msg = """
    Control Your Vehicle!
    ---------------------------
    Moving around:
        W/S: X-Axis
        A/D: Y-Axis
        X/Z: Z-Axis
        J/L: Roll
        M: Zero out applied forces/torques
    CTRL-C to quit
            """
    print(msg)
    move()