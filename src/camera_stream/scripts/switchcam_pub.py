#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import String

def am_die():
    print('die')

if __name__ == '__main__':
    topic_name = '/nautilus/cameras/camera_picker'

    rospy.init_node('keyboard_pub')

    p = rospy.Publisher(topic_name, String, queue_size=10)
    s = String()

    print('select the camera [cam1 or cam2]')
    rospy.on_shutdown(am_die)
    while not rospy.is_shutdown():
        try:
            time.sleep(0.5)
            cam = input("Give Command: ")

            if cam != "cam1" and cam != "cam2":
                print("wow ur bad")
                continue

            s.data = cam

            p.publish(s)
        except:
            break