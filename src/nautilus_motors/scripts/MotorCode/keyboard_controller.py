#!/usr/bin/env python3
import rospy
import time
from geometry_msgs.msg import Wrench

def am_die():
    print('die')

if __name__ == '__main__':
    rospy.init_node('keyboard_pub')

    p = rospy.Publisher('/nautilus/nautilus_motors/wrench', Wrench, queue_size=1)
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
            key = input("Give Key (q to quit): ")

            if (len(key) != 1):
                print('only one character please')
                continue

            if (key == ord('w')):
                w.force.x = min(w.force.x + 0.1, 1.0)
            elif (key == ord('s')):
                w.force.x = max(w.force.x - 0.1, -1.0)
            elif (key == ord('a')):
                w.force.y = min(w.force.y - 0.1, 1.0)
            elif (key == ord('d')):
                w.force.y = max(w.force.y + 0.1, -1.0)
            elif (key == ord('z')):
                w.force.z = max(w.force.z - 0.1, -1.0)
            elif (key == ord('c')):
                w.force.z = min(w.force.z + 0.1, 1.0)
            elif (key == ord('q')):
                w.torque.z = max(w.torque.z - 0.1, -1.0)
            elif (key == ord('e')):
                w.torque.z = min(w.torque.z + 0.1, 1.0)
            elif (key == ord('h')):
                w.force.x, w.force.y, w.force.z = (0,0,0)
                w.torque.z = 0
            p.publish(w)
        except:
            break