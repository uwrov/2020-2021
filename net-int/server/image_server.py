#!/usr/bin/env python3
import json
import rospy
from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit
from geometry_msgs.msg import Twist

HOST_IP = "localhost"
HOST_PORT = "4040"

app = Flask(__name__)
sio = SocketIO(app, cors_allowed_origins="*")

velocity_publisher = None
rate = None
image_subsciber = None
br = None

# CD into the directory src/wb_sol/urdf
# roslaunch gazebo_ros empty_world.launch
# rosrun gazebo_ros spawn_model -file wb.urdf -urdf -model wheely_boi
# To run server
# rosrun wb_sol 2020_server.py
# source devel/setup.sh

def send_image():
    """
    Sends image to client

    Receives movement information from ROS as a image message.
    The image is then converted into a cv2 image and then encoded in vase 64 so
    it can be sent as a JSON object to the client via socket.io

    Parameters
    -------
    data : ROS image
        stores image from ROS

    Returns
    -------
    None
    """
    #rospy.loginfo('Image received...')
    #image = br.imgmsg_to_cv2(data)
    image = cv2.imread("smile.png")
    retval, buffer = cv2.imencode('.png', image)
    img = base64.b64encode(buffer)
    sio.emit("Image Display", {'image': img}, broadcast = True)

@sio.on("Test")
def test():
    print("sending test")
    sio.emit("Test", broadcast = True)

if __name__ == '__main__':
    """ Sets up rospy and starts server """
    # try:
    #     rospy.init_node('wheely_boi', anonymous=True)
    #     #velocity_publisher = rospy.Publisher('/wheely_boi/wheely_boi/cmd', Twist, queue_size=10)
    #     print("starting subsciber")
    #     image_subsciber = rospy.Subscriber("/image/distribute", Image, send_image) # change chatter to url dest
    #     br = CvBridge()
    #     #rate = rospy.Rate(10)
    #     print("starting server")
    #     sio.run(app, host=HOST_IP, port=HOST_PORT)
    # except rospy.ROSInterruptException: pass
    sio.run(app, host=HOST_IP, port=HOST_PORT)
