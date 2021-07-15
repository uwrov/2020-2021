import rospy
# from flask import Flask, render_template
# from flask_socketio import SocketIO, send, emit
import _thread
import time

import image_server
import Move_Server


if __name__ == '__main__':
    """ Sets up rospy and starts servers """
    try:
        print("main server is running")
        rospy.init_node('surface', anonymous=True)
        image_server.start_server()
        Move_Server.start_server()
        time.sleep(5)

        while True:
            choice = input("press q to exit: ")
            print()
            if choice == 'q':
                print('exiting')
                break
    except rospy.ROSInterruptException: pass
