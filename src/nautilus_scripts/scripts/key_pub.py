#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('keyboard_pub')

    p = rospy.Publisher('/buttonPress', String, queue_size=1)

    while True:
        try:
            time.sleep(0.5)
            s = input("Give Key (q to quit): ")
            if (s == 'q'):
                break
            p.publish(String(s))
        except KeyboardInterrupt:
            break