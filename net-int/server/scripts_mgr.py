#!/usr/bin/env python3

import json
import subprocess
import asyncio

from os import listdir
from os.path import isfile, join
from datetime import datetime

from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit

# Dev-environment-only prep commands:
#
# 	Terminal 1:
# 	1. source devel/setup.sh
# 	2. CD into the directory src/wb_sol/urdf
# 	3. roslaunch gazebo_ros empty_world.launch
#
# 	Terminal 2:
# 	1. source devel/setup.sh
# 	2. CD into the directory src/wb_sol/urdf
# 	3. rosrun gazebo_ros spawn_model -file wb.urdf -urdf -model wheely_boi
#
# To run server:
#
#	Separate terminal:
# 	1. Create scripts folder in same directory as server (or modify SCRIPTS_PATH), add all scripts there, sudo chmod +x all of them.
# 	2. Register all files in the scripts path with catkin (edit CMakeLists.txt, then catkin_make)
# 	3. source devel/setup.sh
# 	4. (Only do this in production) roscore
# 	5. ./scripts_mgr.py or python3 scripts_mgr.py


HOST_IP = "localhost"
HOST_PORT = "4046"
SCRIPTS_PATH = '../src/nautilus_scripts/scripts'
# SCRIPTS_PATH = './scripts'
RUNTIME_COMMAND = 'rosrun'
RUNTIME_PARAMS = 'wb_sol'
MAX_LOGS_BUFFER_SIZE = 5

app = Flask(__name__)
sio = SocketIO(app, cors_allowed_origins="*")
scripts = []
logs = []


@sio.on("Send Commands")
def json_request(data):

    """

    Register event to receive JSON commands

    """

    try:
        data = json.loads(data)

        if (not 'command' in data):
            log('error', 'Command not found: ' + str(data))
            return

        #Only one script can be run at a time, can change this to async if needed
        if (data['command'] == 'run'):
            run_script(data)

        elif (data['command'] == 'list'):
            list_scripts()
        else:
            log('error', 'Command not found: ' + str(data))

    except Exception as e:
        log('error', 'Error in command execution: ' + str(e) + ', Command received: ' + str(data))

    emit_logs()


@sio.on("Error Message")
def process_error_msg(data):

    """

    Read and process error messages from client

    """

    try:
        data = json.loads(data)

        if (not 'code' in data):
            log('error', 'Error code not found: ' + str(data))
            return

        log('error', 'Unknown error code received: ' + data['code'] + '. No action taken.')

    except Exception as e:
        log('error', 'Can\'t parse json data: ' + str(data) + '. Exception: ' + str(e))

    emit_logs()


def run_script(data):

    """

    Look for and run a specified script, with parameters. Send back output and ending status.

    """

    if ('arg1' in data and data['arg1'] in scripts):

        if (not 'arg2' in data):
            data['arg2'] = ''
        if (type(data['arg2']) == list):
            data['arg2'] = ' '.join(data['arg2'])

        #Open python script and start reading output
        try:
            proc = subprocess.Popen([RUNTIME_COMMAND, RUNTIME_PARAMS, SCRIPTS_PATH + '/' + data['arg1'], data['arg2']],
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
        except Exception as e:
            log('error', 'Failed to launch script ' + data['arg1'] + '. Exception: ' + str(e))
            return

        #Continuously stream output (synchronously)
        while True:
            output = proc.stdout.readline()

            if output == '' and proc.poll() is not None:
                break
            if output:
                log('script-output', output.strip())

        #Log ending status
        if (proc.returncode == 0):
            log('info', 'Script ' + data['arg1'] + ' executed successfully.')
        else:
            log('error', 'Script ' + data['arg1'] + ' failed with exit code ' + str(proc.returncode))

    else:
        log('error', 'Could not find script.')


def list_scripts():

    """

    Send formatted list of available scripts

    """

    if (len(scripts) > 0):
        log('info', '\n'.join(scripts))
    else:
        log('info', 'No scripts found.')


def log(messageType, message):

    """

    Save log in queue to be sent back to client. If max queue capacity reached, emit all queued up logs to client.

    """

    logs.append({'type': messageType, 'message': message, 'timestamp': str(datetime.now())})

    # Discharge queue if max capacity reached
    if (len(logs) >= MAX_LOGS_BUFFER_SIZE):
        emit_logs()

    print(messageType + ':', message)


def emit_logs():
    """

    Emit all currently queued up logs to client

    """

    sio.emit('Print Console Logs', logs)
    logs.clear()


if __name__ == '__main__':

    # Search configured scripts folder for available scripts & start server
    try:
        scripts = [f for f in listdir(SCRIPTS_PATH) if isfile(join(SCRIPTS_PATH, f))]
        sio.run(app, host=HOST_IP, port=HOST_PORT)
        log('info', 'Script server initialized.')
    except Exception as e:
        log('error', 'Init: Server setup failed' + '. Exception: ' + str(e))
