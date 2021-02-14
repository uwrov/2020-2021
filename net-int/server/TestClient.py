#!/usr/bin/env python3
from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit

HOST_IP = "localhost"
HOST_PORT = "4444"

app = Flask(__name__)
sio = SocketIO(app, cors_allowed_origins="*")

@sio.on("Test")
def get_image(data):
    print("sending test")
    sio.emit("Test", broadcast=True)

if __name__ == '__main__':
    sio.run(app, host=HOST_IP, port=HOST_PORT)
