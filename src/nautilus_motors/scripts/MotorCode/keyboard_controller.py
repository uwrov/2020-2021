#!/usr/bin/env python3
import rospy
import time
from numpy import around
from geometry_msgs.msg import Wrench

def am_die():
    print('die')

if __name__ == '__main__':
    rospy.init_node('keyboard_pub')

    p = rospy.Publisher('/nautilus/thruster_manager/input', Wrench, queue_size=10)
    w = Wrench()

    print('controls:')
    print('w/s -> x (forward/back)')
    print('a/d -> y (left/right)')
    print('z/c -> z (down/up)')
    print('q/e -> r (rot left/rot right)')
    print('h -> zero')
    rospy.on_shutdown(am_die)
    while not rospy.is_shutdown():
        try:
            time.sleep(0.5)
            key = input("Give Key: ")

            if (len(key) != 1):
                print('only one character please')
                continue

            if (key == 'w'):
                w.force.x = around(min(w.force.x + 0.1, 1.0), 1)
            elif (key == 's'):
                w.force.x = around(max(w.force.x - 0.1, -1.0), 1)
            elif (key == 'a'):
                w.force.y = around(min(w.force.y - 0.1, 1.0), 1)
            elif (key == 'd'):
                w.force.y = around(max(w.force.y + 0.1, -1.0), 1)
            elif (key == 'z'):
                w.force.z = around(max(w.force.z - 0.1, -1.0), 1)
            elif (key == 'c'):
                w.force.z = around(min(w.force.z + 0.1, 1.0), 1)
            elif (key == 'q'):
                w.torque.z = around(max(w.torque.z - 0.1, -1.0), 1)
            elif (key == 'e'):
                w.torque.z = around(min(w.torque.z + 0.1, 1.0), 1)
            elif (key == 'h'):
                w.force.x, w.force.y, w.force.z = (0,0,0)
                w.torque.z = 0

            p.publish(w)
        except:
            break
