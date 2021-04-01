#!/usr/bin/env python3
import cv2
import numpy as np

from subway_car import subway_car

RECTANGLE_THRESHOLD = 150
CENTER_THRESHOLD_VARIANCE = 20
PIXELS_PER_CM = 1
LENGTH = 110
HEIGHT = 50
WIDTH = 50
DISPLAY_IMAGE_SCALE = 1
IMAGE_CORNER = (60, 60)
end = (WIDTH * PIXELS_PER_CM, HEIGHT * PIXELS_PER_CM)
side = (LENGTH * PIXELS_PER_CM, HEIGHT * PIXELS_PER_CM)
top = (LENGTH * PIXELS_PER_CM, WIDTH * PIXELS_PER_CM)
sizes = [end, side, end, side, top]
imageSizes = subway_car.scaleTupleArray(sizes, DISPLAY_IMAGE_SCALE)
x_0, y_0 = IMAGE_CORNER
img_width, img_height = imageSizes[0]
img_length, _ = imageSizes[1]
coords = [
    (x_0, y_0),
    (x_0 + img_width, y_0),
    (x_0 + img_width + img_length, y_0),
    (x_0 + (2 * img_width) + img_length, y_0),
    (x_0 + img_width, y_0 - img_height)
    ]

class Photomosaic:
    def __init__(self):
        self.imageIndex = 0
        self.output = np.ones((125, 450, 3), np.uint8) * 255
        self.outputImages = [None] * 5
        
    def add(self, frame):
        """ Adds given frame into the output, returns true if the output
            is ready for presentation
        """
        frame = subway_car.resizeWithAspectRatio(frame, width=800)
        warpedImage, points = subway_car.getRectangleImage(frame.copy(), sizes[self.imageIndex])
        print(warpedImage, points)
        print(imageSizes)
        if (self.imageIndex != len(imageSizes)) and points is not None and warpedImage is not None:
            cv2.drawContours(frame, [points], -1, (0, 0, 255), 4)
            width, height = imageSizes[self.imageIndex]
            x, y = coords[self.imageIndex]
            shrink_img = cv2.resize(warpedImage.copy(), (width, height))
            self.output[y:y + height, x:x + width] = shrink_img

            self.outputImages[self.imageIndex] = warpedImage
            self.imageIndex += 1
        else:
            print('nothing')
        return
    
    def get_output(self):
        scale_percent = 300 # percent of original size
        width = int(self.output.shape[1] * scale_percent / 100)
        height = int(self.output.shape[0] * scale_percent / 100)
        dim = (width, height)
        resized = cv2.resize(self.output.copy(), dim)
        return resized
    
    def is_finished(self):
        return self.imageIndex == len(imageSizes)