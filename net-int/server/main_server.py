#!/usr/bin/env python3
import _thread
import threading

def run_image_server():
    print("running image server")
    exec(open("image_server.py").read())

def run_movement_server():
    print("running movement server")
    exec(open("movement_server.py").read())

if __name__ == '__main__':
    """ Sets up rospy and starts server """
    try:
        t1 = threading.Thread(target=run_image_server)
        t2 = threading.Thread(target=run_movement_server)
        t1.start()
        t2.start()
        # _thread.start_new_thread(run_image_server()) # port 4040
        # _thread.start_new_thread(run_movement_server()) # port 4041
    except:
        print("Threading Error")
