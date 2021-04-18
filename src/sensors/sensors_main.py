#!/usr/bin/python3

import rospy
import master_3 as m3
import distance as dist 
import numpy as np
import time
from Adafruit_BMP085_3 import BMP085

def force(mass, ax, ay, az): 
    return mass * ax, mass * ay, mass * az, mass * netAccel(ax, ay, az) 

def velocity(time, ax, ay, az):
    netA = netAccel(ax, ay, az)
    return time * ax, time * ay, time * az, time * netA

def netAccel(ax, ay, az):
    return np.sqrt((ax * ax) + (ay * ay) + (az * az))

def main():
    d = { "Gx1": 0, "Gy1": 0, "Gz1": 0, 
          "Ax1": 0, "Ay1": 0, "Az1": 0, 
          "An1": 0, "Rx1": 0, "Ry1": 0,  
          "Gx2": 0, "Gy2": 0, "Gz2": 0, 
          "Ax2": 0, "Ay2": 0, "Az2": 0, 
          "An1": 0, "Rx2": 0, "Ry2": 0, 
          "Vx1": 0, "Vy1": 0, "Vz1": 0, "Vn1": 0, 
          "Vx2": 0, "Vy2": 0, "Vz2": 0, "Vn2": 0, 
          "Temp": 0, "Pressure": 0, "Alt": 0,
          "Distance": 0, 
          "Fx1": 0, "Fy1": 0, "Fz1": 0, "Fn1": 0,
          "Fx2": 0, "Fy2": 0, "Fz2": 0, "Fn2": 0}

    # sensor_pub = rospy.Publisher('out/sensors/data', dict, queue_size=1)
    # rospy.init_node('sensor_data')

    while (True): 
        tic = time.perf_counter()

        d["Gx1"], d["Gy1"], d["Gz1"], d["Ax1"], d["Ay1"], d["Az1"]  = m3.mpu6050(0x68)
        d["Rx1"] = m3.get_x_rotation(d["Ax1"], d["Ay1"], d["Az1"])
        d["Ry1"] = m3.get_y_rotation(d["Ax1"], d["Ay1"], d["Az1"])
        d["An1"] = netAccel(d["Ax1"], d["Ay1"], d["Az1"])
        d["Gx2"], d["Gy2"], d["Gz2"], d["Ax2"], d["Ay2"], d["Az2"]  = m3.mpu6050(0x69)
        d["Rx2"] = m3.get_x_rotation(d["Ax2"], d["Ay2"], d["Az2"])
        d["Ry2"] = m3.get_y_rotation(d["Ax2"], d["Ay2"], d["Az2"])
        d["An2"] = netAccel(d["Ax2"], d["Ay2"], d["Az2"])

        time.sleep(0.3)
        toc = time.perf_counter()
        d["Vx1"], d["Vy1"], d["Vz1"], d["Vn1"] = velocity((toc - tic)/1000, d["Ax1"], d["Ay1"], d["Az1"])
        d["Vx2"], d["Vy2"], d["Vz2"], d["Vn2"] = velocity((toc - tic)/1000, d["Ax2"], d["Ay2"], d["Az2"])

        bmp = BMP085(0x77)
        d["Temp"] = bmp.readTemperature()
        d["Pressure"] = bmp.readPressure()
        d["Alt"] = bmp.readAltitude()
        d["Distance"] = dist.distance()
        print(d['Vn1'])

        # sensor_pub.publish(d)

        
if __name__ == '__main__':
    main()
