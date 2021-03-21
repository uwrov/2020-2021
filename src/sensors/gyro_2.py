# -*- coding: utf-8 -*-
"""
Created on Sat Mar  6 16:06:20 2021

@author: squac
"""

#!/usr/bin/python
import smbus
import math
 
# Register
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

# Read 100 times
totalSteps = 100
stepsDone = 0
 
def read_byte(reg):
    return bus.read_byte_data(address, reg)
 
def read_word(add, reg):
    h = bus.read_byte_data(add, reg)
    l = bus.read_byte_data(add, reg+1)
    value = (h << 8) + l
    return value
 
def read_word_2c(add, reg):
    val = read_word(add, reg)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val
 
def dist(a,b):
    return math.sqrt((a*a)+(b*b))
 
def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)
 
def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

 
bus = smbus.SMBus(1) # bus = smbus.SMBus(0) fuer Revision 1

print "Gyro Unit:  Gyro_x |  Gyro_y  |  Gyro_z  |  Acc_x   |  Acc_y   | Acc_z | X Rotation | Y Rotation"
while (stepsDone <= totalSteps):
    #=========================================
    #Gyroscope 1 reading
    address = 0x68       # via i2cdetect
     
    # Activates first Gyroscope
    bus.write_byte_data(address, power_mgmt_1, 0)
     
    #print "Gyroskop One"
    #print "------------------------"

     
    gyro_xout = read_word_2c(address, 0x43)
    gyro_yout = read_word_2c(address, 0x45)
    gyro_zout = read_word_2c(address, 0x47)

    #Properly Scaled
    Gx = gyro_xout / 131
    Gy = gyro_yout / 131
    Gz = gyro_zout / 131

     
    acc_xout = read_word_2c(address, 0x3b)
    acc_yout = read_word_2c(address, 0x3d)
    acc_zout = read_word_2c(address, 0x3f)
     
    #Properly Scaled
    Ax = acc_xout / 16384.0
    Ay = acc_yout / 16384.0
    Az = acc_zout / 16384.0


    print "Gyro 1: ", "%.4f"%Gx, "| %.4f"%Gy, "| %.4f" %Gz, "| %.4f" %Ax, "| %.4f" %Ay, "| %.4f" %Az, "|", get_x_rotation(Ax, Ay, Az), "|", get_y_rotation(Ax, Ay, Az) 
     
    #print "X Rotation: " , get_x_rotation(Ax, Ay, Az)
    #print "Y Rotation: " , get_y_rotation(Ax, Ay, Az)

    #=========================================
    #Gyroscope 2 reading
    address = 0x69       # via i2cdetect
     
    # Activates second Gyroscope
    bus.write_byte_data(address, power_mgmt_1, 0)
    bus.write_byte_data(address, power_mgmt_2, 0)
     
    #print "Gyroskop Two"
    #print "------------------------"
     
    gyro_xout = read_word_2c(address, 0x43)
    gyro_yout = read_word_2c(address, 0x45)
    gyro_zout = read_word_2c(address, 0x47)

    #Properly Scaled
    Gx = gyro_xout / 131
    Gy = gyro_yout / 131
    Gz = gyro_zout / 131

     
    acc_xout = read_word_2c(address, 0x3b)
    acc_yout = read_word_2c(address, 0x3d)
    acc_zout = read_word_2c(address, 0x3f)

    #Properly Scaled 
    Ax = acc_xout / 16384.0
    Ay = acc_yout / 16384.0
    Az = acc_zout / 16384.0

    print "Gyro 2: ", "%.4f"%Gx, "| %.4f"%Gy, "| %.4f" %Gz, "| %.4f" %Ax, "| %.4f" %Ay, "| %.4f" %Az, "|", get_x_rotation(Ax, Ay, Az), "|", get_y_rotation(Ax, Ay, Az) 
     
    #print "X Rotation: " , get_x_rotation(Ax, Ay, Az)
    #print "Y Rotation: " , get_y_rotation(Ax, Ay, Az)
    totalSteps = totalSteps-1
