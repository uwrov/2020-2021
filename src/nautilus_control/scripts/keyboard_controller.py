#!/usr/bin/env python3

import rospy
import curses
from geometry_msgs.msg import Wrench, Vector3

min_thrust = -5 
max_thrust = 5

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

    while not rospy.is_shutdown():
        key = curses.wrapper(getch_c)
        update_func(key, w)

        wrench_publisher.publish(w)
        rate.sleep()

def inc(val):
    return min(val+0.5, max_thrust)

def dec(val):
    return max(val-0.5, min_thrust)

# Returns a Vector3 representing the update to be done to w
def update_func(key, w):
    lin = w.force
    tor = w.torque
    if (key == ord('w')):
        lin.x = inc(lin.x)
    elif (key == ord('s')):
        lin.x = dec(lin.x)
    elif (key == ord('a')):
        lin.y = inc(lin.y)
    elif (key == ord('d')):
        lin.y = dec(lin.y)
    elif (key == ord('x')):
        lin.z = inc(lin.z)
    elif (key == ord('z')):
        lin.z = dec(lin.z)
    elif (key == ord('j')):
        tor.z = dec(tor.z)
    elif (key == ord('l')):
        tor.z = inc(tor.z)
    elif (key == ord('m')):
        lin.x = 0
        lin.y = 0
        lin.z = 0
        tor.z = 0


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