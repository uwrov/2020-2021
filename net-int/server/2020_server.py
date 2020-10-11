import json
# import rospy
from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit
# from geometry_msgs.msg import Twist

HOST_IP = "localhost"
HOST_PORT = "4040"

app = Flask(__name__)
sio = SocketIO(app, cors_allowed_origins="*")

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
    #('Command', command, broadcast=True)
    print('hello')

@sio.on('Send State')
def state(state):
    # state = to_binary(state)
    print_directions(state)

def main():
    # state(movement)
    sio.run(app, host=HOST_IP, port=HOST_PORT)

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

main()
