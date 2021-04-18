#!/usr/bin python3
import pigpio
import rospy
from geometry_msgs.msg import Wrench
from MotorCode.Control import apply_control

"""
ROS motor code driver
"""

keyboard_topic = '/nautilus/nautilus_motors/wrench'
pi = None

def drive(msg):
    w = msg.data
    package = [w.force.x, w.force.y, w.force.z, w.torque.z]
    apply_control(package, pi)

def main():
    pi = pigpio.pi()
    rospy.init_node('motor_driver')
    rospy.Subscriber(keyboard_topic, Wrench, drive)

    rospy.on_shutdown(shutdown_fn)
    rospy.spin()

def shutdown_fn():
    pi.stop()
    print('shutting down')

if __name__ == '__main__':
    main()