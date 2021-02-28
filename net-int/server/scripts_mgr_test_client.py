#!/usr/bin/env python3

import json
import socketio
import shlex
import sys

HOST_IP = "localhost"
HOST_PORT = "4040"
PROMPT = "Command> "
COMMANDS = {'run': 0, 'list': 1, 'error': 3, 'quit': 1, 'help': 1}

sio = socketio.Client(reconnection = False)

@sio.event
def connect():
    print("Connected. Type \'help\' for help.")

    # Only returns execution to sio when the command entered emits event    
    while not command():
        pass

@sio.on("Print Console Logs")
def receive_data(data):
    for row in data:
        print(row["type"] + ":", row["message"], "[" + str(row["timestamp"]) + "]")
    
    if (row["type"] != "script-output"):
        while not command():
            pass

@sio.event
def connect_error(err):
    print("Error: connection failed.")

@sio.event
def disconnect():
    print("Disconnected.")

def command():
    """

    Takes command, parses, and runs it. Returns True when command entered emits socket-io event.

    """

    query = shlex.split(input(PROMPT).strip())

    if (len(query) == 0):
        print("Error: empty command")
        return False
    
    function = query[0]

    if (not function in COMMANDS):
        print("Error: command \'", function, "\' not found.")
        return False
    elif (COMMANDS[function] != 0 and len(query) != COMMANDS[function]):
        print("Error: ", COMMANDS[function] - 1, " arguments required.")
        return False
    elif (function == 'help'):
        print("List of commands: " + str(list(COMMANDS.keys())))
        print("Put spaces between arguments. Quoted arguments are allowed.")
        return False
    
    payload = {}
    
    if (function == 'quit'):
        sio.disconnect()
        return True
    elif (function == 'error'):
        event = 'Error Message'
        payload["code"] = query[1]
        payload["error"] = query[2]
    else:
        event = 'Send Commands'
        payload["command"] = function
        
        if (len(query) > 1):
            payload["arg1"] = query[1]
            payload["arg2"] = query[2:]

    sio.emit(event, json.dumps(payload))  
    return True    

if __name__ == '__main__':

    print("Script Manager Test Client")
    try:
    	sio.connect('http://' + HOST_IP + ':' + HOST_PORT)
    except:
        print('Client quit.')
        sys.exit()
