#!/usr/bin/env python3 

import RPi.GPIO as gp
import os
import cv2 as cv 
import numpy as np
import time
import rospy
from sensor_msgs.msg import CompressedImage

class MultiAdapter:
    camNum = 2
    adapter_info = {   "A":{   "i2c_cmd":"i2cset -y 0 0x70 0x00 0x04",
                                    "gpio_sta":[0,0,1],
                            },
                        "B":{
                                "i2c_cmd":"i2cset -y 0 0x70 0x00 0x05",
                                "gpio_sta":[1,0,1],
                            },
                        "C":{
                                "i2c_cmd":"i2cset -y 0 0x70 0x00 0x06",
                                "gpio_sta":[0,1,0],
                            },
                        "D":{
                                "i2c_cmd":"i2cset -y 0 0x70 0x00 0x07",
                                "gpio_sta":[1,1,0],
                            },
                     } 
    camera = cv.VideoCapture(0) 
    width = 320
    height = 240 
   
    def __init__(self):
       gp.setwarnings(False)
       gp.setmode(gp.BOARD)
       gp.setup(7, gp.OUT)
       gp.setup(11,gp.OUT)
       gp.setup(12,gp.OUT)

    def init_channel(self,index):
        channel_info = self.adapter_info.get(index)
        if channel_info == None:
            print("Can't get this info")
        os.system(channel_info["i2c_cmd"]) # i2c write
        gpio_sta = channel_info["gpio_sta"] # gpio write
        gp.output(7, gpio_sta[0])
        gp.output(11, gpio_sta[1])
        gp.output(12, gpio_sta[2])

    def select_channel(self,index):
        channel_info = self.adapter_info.get(index)
        if channel_info == None:
            print("Can't get this info")
        gpio_sta = channel_info["gpio_sta"] # gpio write
        gp.output(7, gpio_sta[0])
        gp.output(11, gpio_sta[1])
        gp.output(12, gpio_sta[2])

    def init(self,width,height):
        for i in range(self.camNum):
           self.height = height
           self.width = width
           self.init_channel(chr(65+i)) 
           self.camera.set(3, self.width)
           self.camera.set(4, self.height)
           ret, frame = self.camera.read()
           if ret == True:
               print("camera %s init OK" %(chr(65+i)))
               pname = "image_"+ chr(65+i)+".jpg"
               cv.imwrite(pname,frame)
               time.sleep(1)
           else:
               print("camera %s init ERROR" %(chr(65+i)))
               self.camera.release()

    def preview(self):
        font                   = cv.FONT_HERSHEY_PLAIN
        fontScale              = 1
        fontColor              = (255,255,255)
        lineType               = 1
        factor  = 20
        black = np.zeros(((self.height+factor)*2, self.width*2, 3), dtype= np.uint8) 
        i = 0
        rospy.init_node("testNode")
        pub = rospy.Publisher('/camera/image_raw', CompressedImage, queue_size=10)
        c = CompressedImage()
        c.format = 'jpeg'
        rospy.on_shutdown(lambda: self.camera.release())
        while not rospy.is_shutdown():
            try:
                self.select_channel(chr(65+i)) 
                ret, frame = self.camera.read()
                i = i+1
                if i==self.camNum:
                    i = 0
                c.data = np.array(cv.imencode('.jpeg', frame)[1]).tostring()
                pub.publish(c)
                time.sleep(0.1)
            except Exception as e:
                self.camera.release()
                print(e)
                break

if __name__ == '__main__':
    multi = MultiAdapter()
    multi.init(320,240)
    multi.preview()