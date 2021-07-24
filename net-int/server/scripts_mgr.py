#!/usr/bin/env python3

import json
import subprocess
import asyncio

from os import listdir
from os.path import isfile, join
from datetime import datetime



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

SCRIPTS_PATH = '../../src/nautilus_scripts/scripts/'
# SCRIPTS_PATH = './scripts'
RUNTIME_COMMAND = 'rosrun'
RUNTIME_PARAMS = 'nautilus_scripts'
MAX_LOGS_BUFFER_SIZE = 5

class ScriptManager:

    def __init__(self, sio):
        self.sio = sio
        self.scripts = [f for f in listdir(SCRIPTS_PATH) if isfile(join(SCRIPTS_PATH, f))]
        self.logs = []

    def json_request(self, data):

        """

        Register event to receive JSON commands

        """

        try:
            data = json.loads(data)

            if (not 'command' in data):
                self.log('error', 'Command not found: ' + str(data))
                return

            #Only one script can be run at a time, can change this to async if needed
            if (data['command'] == 'run'):
                self.run_script(data)
            elif (data['command'] == 'list'):
                self.list_scripts()
            else:
                self.log('error', 'Command not found: ' + str(data))

        except Exception as e:
            self.log('error', 'Error in command execution: ' + str(e) + ', Command received: ' + str(data))

        self.emit_logs()

    def process_error_msg(self, data):

        """

        Read and process error messages from client

        """

        try:
            data = json.loads(data)

            if (not 'code' in data):
                self.log('error', 'Error code not found: ' + str(data))
                return

            self.log('error', 'Unknown error code received: ' + data['code'] + '. No action taken.')

        except Exception as e:
            self.log('error', 'Can\'t parse json data: ' + str(data) + '. Exception: ' + str(e))

        self.emit_logs()

    def run_script(self, data):

        """

        Look for and run a specified script, with parameters. Send back output and ending status.

        """

        if ('arg1' in data and data['arg1'] in self.scripts):

            if (not 'arg2' in data):
                data['arg2'] = ''
            if (type(data['arg2']) == list):
                data['arg2'] = ' '.join(data['arg2'])

            #Open python script and start reading output
            try:
                proc = subprocess.Popen([RUNTIME_COMMAND, RUNTIME_PARAMS, SCRIPTS_PATH + '/' + data['arg1'], data['arg2']],
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
            except Exception as e:
                self.log('error', 'Failed to launch script ' + data['arg1'] + '. Exception: ' + str(e))
                return

            #Continuously stream output (synchronously)
            while True:
                output = proc.stdout.readline()

                if output == '' and proc.poll() is not None:
                    break
                if output:
                    self.log('script-output', output.strip())

            #Log ending status
            if (proc.returncode == 0):
                self.log('info', 'Script ' + data['arg1'] + ' executed successfully.')
            else:
                self.log('error', 'Script ' + data['arg1'] + ' failed with exit code ' + str(proc.returncode))

        else:
            self.log('error', 'Could not find script.')

    def list_scripts(self):

        """

        Send formatted list of available scripts

        """

        if (len(self.scripts) > 0):
            self.log('info', '\n'.join(self.scripts))
        else:
            self.log('info', 'No scripts found.')

    def log(self, messageType, message):

        """

        Save log in queue to be sent back to client. If max queue capacity reached, emit all queued up logs to client.

        """

        self.logs.append({'type': messageType, 'message': message, 'timestamp': str(datetime.now())})

        # Discharge queue if max capacity reached
        if (len(self.logs) >= MAX_LOGS_BUFFER_SIZE):
            self.emit_logs()

        print(messageType + ':', message)

    def emit_logs(self):
        """

        Emit all currently queued up logs to client

        """

        self.sio.emit('Print Console Logs', self.logs)
        self.logs.clear()
