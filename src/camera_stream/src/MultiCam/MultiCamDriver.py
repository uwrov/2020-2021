#!/usr/bin/env python3

import RPi.GPIO as gp
import os
import cv2

class MultiAdapter:
    camNum = 2
    adapter_info = {
        "A": {"i2c_cmd": "i2cset -y 0 0x70 0x00 0x04",
              "gpio_sta": [0, 0, 1],
              },
        "B": {
            "i2c_cmd": "i2cset -y 0 0x70 0x00 0x05",
            "gpio_sta": [1, 0, 1],
        },
        "C": {
            "i2c_cmd": "i2cset -y 0 0x70 0x00 0x06",
            "gpio_sta": [0, 1, 0],
        },
        "D": {
            "i2c_cmd": "i2cset -y 0 0x70 0x00 0x07",
            "gpio_sta": [1, 1, 0],
        },
    }
    width = 640
    height = 480
    camera = None

    # constructor
    def __init__(self, camAddress=0):
        print(camAddress)
        self.camera = cv2.VideoCapture(-1, cv2.CAP_V4L2)
        print(camAddress)
        gp.setwarnings(False)
        gp.setmode(gp.BOARD)
        gp.setup(7, gp.OUT)
        gp.setup(11, gp.OUT)
        gp.setup(12, gp.OUT)

    # ===========================
    # Private functions
    # ===========================

    def _set_pins(self, inputs):
        gp.output(7, inputs[0])
        gp.output(11, inputs[1])
        gp.output(12, inputs[2])

    def _init_channel(self, index):
        channel_info = self.adapter_info.get(index)
        print("initializing picam", str(index))
        if channel_info == None:
            raise Exception(f"Can't get infor for picam {index}")
        os.system(channel_info["i2c_cmd"])  # i2c write
        self._set_pins(channel_info['gpio_sta'])

    # ===========================
    # Public functions
    # ===========================
    # Start the multi camera adapter, assumes
    # valid inputs for nCams, width, and height
    def start_cams(self, nCams=2, width=640, height=480):
        self.height = height
        self.width = width
        self.camNum = nCams

        for i in range(self.camNum):
            self._init_channel(chr(65+i))
            self.camera.set(3, self.width)
            self.camera.set(4, self.height)
            ret, frame = self.camera.read()
            if not ret:
                print("camera %s init ERROR" % (chr(65+i)))
                self.camera.release()

    # Switch the camera currently viewed by the adapter
    def select_channel(self, index):
        channel_info = self.adapter_info.get(index)
        if channel_info == None:
            print("Can't get this info")
        self._set_pins(channel_info['gpio_sta'])

    # Get the current frame
    def get_frame(self):
        return self.camera.read()

    # cleanup, please call this to ensure everything is gonna work fine
    def stop_cams(self):
        self.camera.release()
