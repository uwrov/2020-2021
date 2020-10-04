import json
from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit

HOST_IP = "localhost"
HOST_PORT = "4040"

app = Flask(__name__)
sio = SocketIO(app, cors_allowed_origins="*")

@sio.on("Send Movement")
def send_movement():
    emit('Movement', {"speed": speed, "direction": direction}, broadcast=True)

@sio.on('Send Sensor Data')
def send_sensor_data():
    emit('Senor Data', {'sensor data': sensor_data}, broadcast=True)

@sio.on('Send Command')
def send_command(command):
    emit('Command', command, broadcast=True)
