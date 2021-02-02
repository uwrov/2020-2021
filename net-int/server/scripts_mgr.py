#!/usr/bin/env python3

import json
import subprocess
import asyncio

from os import listdir
from os.path import isfile, join
from datetime import datetime

from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit

# CD into the directory src/wb_sol/urdf
# roslaunch gazebo_ros empty_world.launch
# rosrun gazebo_ros spawn_model -file wb.urdf -urdf -model wheely_boi
#
# To run server
# Register all files in the scripts path with catkin
# source devel/setup.sh
# roscore
# ./scripts_mgr.py


HOST_IP = "localhost"
HOST_PORT = "4040"
SCRIPTS_PATH = './scripts'
RUNTIME_COMMAND = 'rosrun wb_sol'

app = Flask(__name__)
sio = SocketIO(app, cors_allowed_origins="*")
scripts = []


@sio.on("Send Commands")
def json_request(sid, data):
    
"""

Register event to receive JSON commands

"""

    try:
        data = json.loads(data)

        if (not 'command' in data):
            log('error', 'Command not found: ' + data)
            return

        #Only one script can be run at a time, can change this to async if needed
        if (data['command'] == 'run'): 
            run_script(data)
            
        else if (data['command'] == 'list'):
            list_scripts()
        else:
            log('error', 'Command not found: ' + data)
        
    except:
        log('error', 'Can\'t parse json data: ' + data)


@sio.on("Error Message")
def process_error_msg(sid, data):
    
"""

Read and process error messages from client

"""

    try:
        data = json.loads(data)

        if (not 'code' in data):
            log('error', 'Error code not found: ' + data)
            return
        
        log('error', 'Unknown error code received: ' + data['code'] + '. No action taken.')
        
    except:
        log('error', 'Can\'t parse json data: ' + data)


def run_script(data):
    
"""

Look for and run a specified script, with parameters. Send back output and ending status.

"""

    if ('arg1' in data and data['arg1'] in scripts):

        if ('arg2' not in data)
            data['arg2'] = ''
        if (data['arg2'] is list)
            data['arg2'] = ' '.join(data['arg2'])
            
        #Open python script and start reading output
        proc = subprocess.Popen([RUNTIME_COMMAND, SCRIPTS_PATH + '/' + data['arg1'], data['arg2']], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        output = proc.stdout.readline()

        #Continuously stream output (synchronously)
        while output != '' or process.poll() is None:
            if output:
                log('script-output', output.strip())
                
            rc = process.poll()
            output = proc.stdout.readline()

        #Log ending status
        if (proc.returncode == 0):
            log('info', 'Script ' + data['arg1'] + ' executed successfully.')
        else:
            log('error', 'Script ' + data['arg1'] + ' failed with exit code ' + proc.returncode)
            
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

Send message back to client

"""

    sio.emit('Print Console Logs', [  {'type': messageType, 'message': message, 'timeStamp': datetime.now()}  ])


if __name__ == '__main__':

    # Search configured scripts folder for available scripts & start server
    try:
        scripts = [f for f in listdir(SCRIPTS_PATH) if isfile(join(SCRIPTS_PATH, f))]
        sio.run(app, host=HOST_IP, port=HOST_PORT)
        log('info', 'Script server initialized.')
    except:
        log('error', 'Init: Server setup failed')


