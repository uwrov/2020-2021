#!/usr/bin/env python3

from MultiCam.MultiCamDriver import MultiAdapter
import cv2

class MultiCamManager:
    usbcam_stream = None
    picam_manager = None
    useUsbCam = False
    curr_channel = 1
    nPiCam = 0

    flip_camera_dict = {
        0: False,   # USB Camera
        1: False,   # PiCamera A
        2: False,   # PiCamera B
        3: False,   # PiCamera C
        4: False    # PiCamera D
    }

    def __init__(self, nPiCam=2, useUsbCam=False,
                usbLocation='/dev/v4l/by-id/usb-HBVCAM_CAMERA_HBVCAM_CAMERA-video-index0',
                picamWidth=640, picamHeight=480,
                usbWidth=640, usbHeight=480,
                framerate=20):
        # set up usb cameras if they are wanted
        try:
            if useUsbCam:
                self.usbcam_stream = cv2.VideoCapture(usbLocation)
                self.usbcam_stream.set(3, usbWidth)
                self.usbcam_stream.set(4, usbHeight)
                self.usbcam_stream.set(5, framerate)
                self.useUsbCam = useUsbCam

            self.picam_manager = MultiAdapter()
            self.picam_manager.start_cams(nPiCam, picamWidth, picamHeight, framerate)
            self.nPiCam = nPiCam
        except:
            if useUsbCam and self.usbcam_stream is not None:
                self.usbcam_stream.release()

    # 0 : usb camera
    # 1 - 4 : picameras [A-D]
    # returns (ret, frame)
    def read_channel(self, channel):
        if channel == 0:
            if not self.useUsbCam:
                raise Exception("USB Camera not enabled")
            return self.usbcam_stream.read()
        
        if channel > self.nPiCam:
            raise Exception("Picam index out of bounds")
        self.picam_manager.select_channel(chr(65 + (channel - 1)))
        return self.picam_manager.get_frame()

    def read_current_channel(self):
        ret, frame = self.read_channel(self.curr_channel)
        if (self.flip_camera_dict[self.curr_channel]):
            frame = cv2.flip(frame, 1)
        
        return ret, frame

    def set_current_channel(self, channel):
        if channel == 0 and not self.useUsbCam:
            raise Exception("USB Camera not enabled")

        if channel > self.nPiCam:
            raise Exception("Picam index out of bounds")

        print("switching to channel", str(channel))
        self.curr_channel = channel

    def close(self):
        if self.useUsbCam:
            self.usbcam_stream.release()
        self.picam_manager.stop_cams()

    def cam_switch_callback(self, msg):
        self.set_current_channel(msg.data)