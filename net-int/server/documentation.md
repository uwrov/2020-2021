# UWROV Surface Server Documentation

## **Contents:**

- Description | 1
- Running the Server | 1
- Socket-io Interface | 1

### **Description:**

This document outlines the specifications for the server that will be running on the surface system. The server will act as an intermediary between the socket-io library and the ROS network system. This server will be written in Python in order to communicate between ROS and the Web App smoothly.

### **Running Server:**

To start the server, run the bash file &quot;run.sh&quot;. This will start all the main server along with the UI and roscore. After running all the servers the bash script will end. You will need to close all the terminals yourself afterwards.

To add another file copy

```

gnome-terminal -- bash -c 'source devel/setup.sh; cd PATH/TO/FILE; python3 YOUR_SERVER.py; $SHELL'

```

Replace PATH/TO/FILE with the correct path to file and YOUR\_SERVER with the Python server you want to run. If your file or file takes a significant amount of time to start, consider adding

```

sleep X

```

With X being the time in seconds, so that your file can finish starting up before running the next script.

Note: since this command uses gnome-terminal to create a new terminal, you can make the terminal run any script you like.

Note: depending on your file, you may not need to source devel/setup.sh before pathing to your file.

Note: If your server is in the src folder, make you add the server to the CMakeLists.txt, cmake the repo and make your script executable before attempting to add your server to this bash file.

### **Socket-io Interface:**

| Module | Port Number |
| --- | --- |
| Main | **4040** |

All modules run on port 4040

**Dependencies**:

- Socket-io, Flask-socket-io

**Description**:

The main server will use socket-io to communicate between the web app (client) and this server. The main server consists of 3 modules:

- Movement Server
- Image Server
- Scripts Server

All socket.on(''), receive, statements for will be in the main server.


### **Controller (Movement Server) API (S.IO):**

| Event Name | Data type Structure| Description | Receive or Send|
| --- | --- | --- | --- |
| "Send State" | JSON / Dictionary | Receives movement information from controller as a JSON object.The movement info is then converted into a Wrench object and published to the rospy. <br> <br> JSON Format: <br> { <br> "lin_x": 0, <br> "lin_y": 0, <br> "lin_z": 0, <br> "ang_x": 0, <br> "ang_y": 0,<br> "ang_z": 0 <br> "a": false, <br> "b": false, <br> "x": false, <br> "y": false <br>} | Receive |


### **Image display API (S.IO):**

| Event Name | Data type Structure |Description |Receive or Send|
| --- | --- | --- | --- |
| "Image Display" | JSON Object | Sends a JSON object: <br> { <br> "Image": img, <br> "Id": id <br> } <br> To the client to display an image or camera feed. "Image" stores the actual image to be displayed, "Id" indicates to the client which component to use. | Send |
| "Get IDs" | None | Client emits event to notify the server that the client wants a list of camera ids. This socket takes in no arguments. | Receive |
| "IDs" | JSON Object | The server sends a list of camera IDs to client in reponse to "Get IDs" event call <br><br> JSON object Format: <br> { <br> "ids": ids <br> } <br><br> Where "ids" is the list of camera IDs. All Ids are strings EX: <br> ["front_cam", "back_cam", ...] | Send |
|"Set Camera" | TBD | TBD | Send |



### **Console interface:**

| Event Name | Data type Structure | Description | Receive or Send |
| --- | --- | --- | --- |
| "Print Console Logs "| Array of JSON objects | Formats all the logs to the list of JSON objects: <br> [ <br> { <br> "type" : "error/info", <br> "message" : "log message" <br> "timestamp": "timestamp" <br>}, <br> { ... }, { ... } ... <br> ] <br> then sends them to the console component where they will be formatted and printed. Logs will be printed in the order of the objects in the list. | Send |
| "Error Message" | JSON object | Contains info on error sent from console in Json format: <br> { <br> "code" : 123,<br> "error" : "error description here" <br>} <br> Prints the error to internal server logs, and parses the code to take action on the error (resend something to console, or try to reopen the last file etc.) | Receive |
| "Send Commands" | String | Commands are received from the console component. Commands can do different things. For example (/run filename.py) to run some file from the server. <br> JSON format: <br>{ <br> "command" : "run/list", <br> "arg1" : "filename.py",<br> "arg2" : ["a", "b", …]<br>}.<br> Json will be parsed and the common will be executed based on the "command". <br> <br> If something goes wrong the server will send console logs back to the console component. It is the console's job to check the syntax correctness of the command, but the server will see if the command (fails / succeeds) by executing it. <br><br> For example: <br> This is the Json received: <br>{ <br> "command" : "run", <br> "arg1" : "test.py" <br> } <br> The server tries to run it but test.py does not exist. The server will then create a log: <br> { <br> "type" : "error", <br> "message" : "Failed executing 'run test.py': file not found", <br> "timestamp": "xxxx-xx-xx hh:mm:ss.xxxxxxx" <br> } <br> and send it to the console through "Send Console Logs" described above. Or if the command succeeds the server will create a log: <br>{ <br> "type" : "info", <br> "message" : "Command 'run test.py' executed successfully", <br> "timestamp": "xxxx-xx-xx hh:mm:ss.xxxxxxx" <br> } <br> and send it to the console through "Send Console Logs" described above. | Receive |
