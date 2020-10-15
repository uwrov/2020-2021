#!/usr/bin/env python3
import json
import rospy
from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit
from geometry_msgs.msg import Twist

HOST_IP = "localhost"
HOST_PORT = "4040"

app = Flask(__name__)
sio = SocketIO(app, cors_allowed_origins="*")

#@sio.on("Send Movement")
def send_movement():
    #emit('Movement', {"speed": speed, "direction": direction}, broadcast=True)
    distance = 3
    speed = 1
    rospy.init_node('wheely_boi', anonymous=True)
    pub_boi = rospy.Publisher('chatter', Twist, queue_size=10)
    msg = Twist()
    msg.linear.x = 1
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0
    while not rospy.is_shutdown():
        t0 = rospy.Time.now().to_sec()
        current_dist = 0
        print(msg.linear.x)
        while (current_dist < distance):
            print(current_dist)
            print(distance)
            pub_boi.publish(msg)
            t1 = rospy.Time.now().to_sec()
            print(t0)
            print(t1)
            current_dist = speed * (t1 - t0)
        msg.linear.x = 0
        print(msg.linear.x)
        pub_boi.publish(msg)
        rospy.signal_shutdown('task done')

#send data as a dicitonary wtih keys: lx, ly,lz, ax, ay, az
# the values will be the respective values for the Keys
#send as twist object
@sio.on("Send Movement")
def send_movement(data):
    #('Movement', {"speed": speed, "direction": direction}, broadcast=True)
    msg = Twist()
    msg.linear.x = data

def send_sensor_data():
    emit('Senor Data', {'sensor data': sensor_data}, broadcast=True)

@sio.on('Send Command')
def send_command(command):
    #emit('Command', command, broadcast=True)
    print(hello)

if __name__ == '__main__':
    try:
	sio.run(app, host=HOST_IP, port=HOST_PORT)
        send_movement()
    except rospy.ROSInterruptException: pass
    #('Command', command, broadcast=True)
    print('hello')

@sio.on('Send State')
def state(state):
    # state = to_binary(state)
    print_directions(state)

def to_binary(state):
    for direction in state:
        if state[direction]:
            state[direction] = 1
        else:
            state[direction] = 0
    return state

def print_directions(state):
    for direction in state:
        print(str(direction) + " = " + str(state[direction]))
