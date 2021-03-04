from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit

HOST_IP = "localhost"
HOST_PORT = "4040"

app = Flask(__name__)
sio = SocketIO(app, cors_allowed_origins="*")


if __name__ == '__main__':
    print("strarting sdfa")
    sio.run(app, host=HOST_IP, port=HOST_PORT)
    print("big pp")
