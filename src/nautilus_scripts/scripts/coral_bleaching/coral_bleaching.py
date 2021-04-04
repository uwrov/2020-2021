#!/user/bin/env python3
"""
ROS driver for the subway car photomosaic task,
orignal code authored by
Peyton Lee, Margot Adam, Cindy Zou and Jonathan Wong
"""

import cv2
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import _Image

