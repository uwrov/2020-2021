### **Image display API (ROS):**

| **ROS Topic** | **Data type**** Structure **|** Description **|** Publisher or Subscriber** |
| --- | --- | --- | --- |
| /images/distribute | CompressedImage | Takes in a ROS compressed image message then sends it to the client as a JSON Object via socket.io <br> <br> JSON Object Format: <br> { <br> &quot;Image&quot;: img, <br> &quot;Id&quot;: id <br> }  <br><br> Where &quot;img&quot; is the compressed image. And id uniquely identifies this ROS topic | Subscriber |
| &quot;/nautilus/nautilus/camera1/nautilus\_cam/compressed&quot; | CompressedImage | Same as adobe with different id. | Subscriber |
| &quot;/nautilus/nautilus/camera2/nautilus\_cam/compressed&quot; | CompressedImage | Same as above with different id. | Subscriber |

### **ROS Interface:**

Description: The ROS interface will communicate information between the server and the Robot System on the Pi. This will include launching ROS nodes that will contain autonomous commands.
