4

# UWROV Surface Server Documentation

## **Contents:**

- Description | 1
- Running the Server | 1
- Socket-io Interface | 1

### **Description:**

This document outlines the specifications for the server that will be running on the surface system. The server will act as an intermediary between the socket-io library and the ROS network system. This server will be written in Python in order to communicate between ROS and the Web App smoothly.

### **Running Server:**

To start the server, run the bash file &quot;run.sh&quot;. This will start all the servers in the scripts file. After running all the servers the bash script will end. You will need to close all the terminals yourself afterwards.

To add another server copy

```

gnome-terminal -- bash -c &#39;source devel/setup.sh; cd PATH/TO/FILE; python3 YOUR\_SERVER.py; $SHELL&#39;.

```

Replace PATH/TO/FILE with the correct path to file and YOUR\_SERVER with the Python server you want to run. If your server or file takes a significant amount of time to start, consider adding

```

sleep X

```

With X being the time in seconds, so that your file can finish starting up before running the next script.

Note: since this command uses gnome-terminal to create a new terminal, you can make the terminal run any script you like.

Note: If your server is in the src folder, make you add the server to the CMakeLists.txt, cmake the repo and make your script executable before attempting to add your server to this bash file.

### **Socket-io Interface:**

| **Module** | **Port Number** |
| --- | --- |
| Console | **4046** |
| Image | **4040** |
| Controller | **4041** |
| Scripts | **4046** |

Dependencies:

- Socket-io, Flask-socket-io

Description: Each surface server will use socket-io to communicate between the web app (client) and this server.

- Movement Server
- Image Server
- Scripts Server

### **Controller (Movement Server) API (S.IO):**

| **Event Name** | Data type Structure| Description | Receive or Send|
| --- | --- | --- | --- |
| Send State | JSON / Dictionary | Receives movement information from controller as a JSON object.The movement info is then converted into a Wrench object and published to the rospy. JSON Format <br> { <br> &quot;lin\_x&quot;: 0, <br> &quot;lin\_y&quot;: 0, <br>&quot;lin\_z&quot;: 0,<br> &quot;ang\_x&quot;: 0,<br> &quot;ang\_y&quot;: 0,<br> &quot;ang\_z&quot;: 0 <br> } | Receive |

### **Image display API (S.IO):**

| **Event Name** | Data type Structure |Description |Receive or Send|
| --- | --- | --- | --- |
| &quot;Image Display&quot; | JSON Object | Sends a JSON object: <br> { <br> &quot;Image&quot;: img, <br> &quot;Id&quot;: id <br> } <br> To the client to display an image or camera feed. &quot;Image&quot; stores the actual image to be displayed, &quot;Id&quot; indicates to the client which component to use. | Send |
| &quot;Get IDs&quot; | None | Client emits &quot;Get IDs&quot; to notify the server that the client wants a list of camera ids. This socket takes in no arguments. | Receive |
| &quot;IDs&quot; | Json Object | The server sends a list of camera IDs to client <br><br> JSON object Format: <br> { <br> &quot;ids&quot;: ids <br> } <br><br> Where &quot;ids&quot; is the list of camera IDs. All Ids are strings EX: <br> [&quot;front\_cam&quot;, &quot;down\_cam&quot;, ...] | Send |

### **Image display API (ROS):**

| **ROS Topic** | **Data type**** Structure **|** Description **|** Publisher or Subscriber** |
| --- | --- | --- | --- |
| /images/distribute | CompressedImage | Takes in a ROS compressed image message then sends it to the client as a JSON Object via socket.io <br> <br> JSON Object Format: <br> { <br> &quot;Image&quot;: img, <br> &quot;Id&quot;: id <br> }  <br><br> Where &quot;img&quot; is the compressed image. And id uniquely identifies this ROS topic | Subscriber |
| &quot;/nautilus/nautilus/camera1/nautilus\_cam/compressed&quot; | CompressedImage | Same as adobe with different id. | Subscriber |
| &quot;/nautilus/nautilus/camera2/nautilus\_cam/compressed&quot; | CompressedImage | Same as above with different id. | Subscriber |

### **Console interface:**

Interface:

| **Event Name** | **Data type**** Structure **|** Description **|** Receive or Send** |
| --- | --- | --- | --- |
| &quot;Print Console Logs&quot; | Array of json objects | Formats all the logs to the list of Json objects: <br> [ <br> { <br> &quot;type&quot; : &quot;error/info&quot;, <br> &quot;message&quot; : &quot;log message&quot; <br> &quot;timestamp&quot;: &quot;timestamp&quot; <br>}, <br>{ ... }, { ... } ... <br> ] <br>then sends them to the console component where they will be formatted and printed. Logs will be printed in the order of the objects in the list. | Send |
| &quot;Error Message&quot; | Json object | Contains info on error sent from console in Json format: <br> { <br> &quot;code&quot; : 123,<br> &quot;error&quot; : &quot;error description here&quot; <br>} <br> Prints the error to internal server logs, and parses the code to take action on the error (resend something to console, or try to reopen the last file etc.) | Receive |
| &quot;Send Commands&quot; | String | Commands are received from the console component. Commands can do different things. For example (/run filename.py) to run some file from the server. <br> Json format: <br>{ <br> &quot;command&quot; : &quot;run/list&quot;, <br> &quot;arg1&quot; : &quot;filename.py&quot;,<br> &quot;arg2&quot; : [&quot;a&quot;, <br> &quot;b&quot;, …]<br>}.<br>Json will be parsed and the common will be executed based on the &quot;command&quot;. If something goes wrong the server will send console logs back to the console component. It is the console&#39;s job to check the syntax correctness of the command, but the server will see if the command (fails / succeeds) by executing it.For example:This is the Json received:{ &quot;command&quot; : &quot;run&quot;, &quot;arg1&quot; : &quot;test.py&quot;}The server tries to run it but test.py does not exist. The server will then create a log:{ &quot;type&quot; : &quot;error&quot;, &quot;message&quot; : &quot;Failed executing &#39;run test.py&#39;: file not found&quot; &quot;timestamp&quot;: &quot;xxxx-xx-xx hh:mm:ss.xxxxxxx&quot;}and send it to the console through &quot;Send Console Logs&quot; described above.Or if the command succeeds the server will create a log:{ &quot;type&quot; : &quot;info&quot;, &quot;message&quot; : &quot;Command &#39;run test.py&#39; executed successfully&quot; &quot;timestamp&quot;: &quot;xxxx-xx-xx hh:mm:ss.xxxxxxx&quot;}and send it to the console through &quot;Send Console Logs&quot; described above. | Receive |


### **ROS Interface:**

Description: The ROS interface will communicate information between the server and the Robot System on the Pi. This will include launching ROS nodes that will contain autonomous commands.
