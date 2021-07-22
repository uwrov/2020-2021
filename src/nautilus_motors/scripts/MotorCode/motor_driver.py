#!/usr/bin/env python3
import pigpio
import rospy
from geometry_msgs.msg import Wrench
from MotorCode.Control import apply_control

"""
ROS motor code driver
"""

#keyboard_topic = '/nautilus/nautilus_motors/wrench'
command_topic = '/nautilus/motors/commands'
pi = pigpio.pi()

def drive(w):
    package = [w.force.x, w.force.y, w.force.z, w.torque.z]
    print(package)
    assert pi is not None
    apply_control(package, pi)

    for i in (21, 20, 16, 12, 26, 19):
        print(pi.get_servo_pulsewidth(i), end=', ')
    print()

def main():
    print('starting listener on', command_topic)
    rospy.init_node('motor_driver')
    rospy.Subscriber(command_topic, Wrench, drive)

    rospy.on_shutdown(shutdown_fn)
    rospy.spin()

def shutdown_fn():
    pi.stop()
    print('shutting down')

if __name__ == '__main__':
    main()
