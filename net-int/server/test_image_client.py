#!/usr/bin/env python3
import socketio

HOST_IP = "localhost"
HOST_PORT = "4040"


sio = socketio.Client()

@sio.on('IDs')
def print_this(data):
    print('hello')
    print(data)
@sio.on('hello')
def hey(data):
    print(data)

@sio.on('connect')
def on_connect():
    sio.emit('Get IDs')

if __name__ == '__main__':
    """ Sets up rospy and starts server """
    sio.connect('http://'+HOST_IP+":"+HOST_PORT)
