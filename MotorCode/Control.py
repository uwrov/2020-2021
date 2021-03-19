import pigpio
import curses
import time
import numpy as np

# Placeholder values giving a range for use in roation.
min = 500
max = 2500
slope = (max - min) / 2
intercept = max - slope * (1)

inputs = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 0]]
pin = 21
pin2 = 20
pin3 = 16
pin4 = 12
pin5 = 26
pin6 = 19

# Allows the control of motors given an input array and an output array.
def control(inputArr):
    output = [0, 0, 0, 0, 0, 0]
    # Checks if the given input array is valid and shows an error message if not.
    for item in inputArr:
        if item < -1 or item > 1:
            print("Error")
    # Case where there is no rotation based on the "R" value of the input array being zero.
    if (inputArr[3] == 0):
        output[4] = (slope * inputArr[2]) + intercept
        output[5] = (slope * inputArr[2]) + intercept
        c1 = (slope * 0.5 * (inputArr[0] + inputArr[1])) + intercept
        c2 = (slope * 0.5 * (((-1) * inputArr[0]) + inputArr[1])) + intercept
        output[0] = c1
        output[1] = c2
        output[2] = c1
        output[3] = c2
    # Case where there is rotation based on the "R" value of the input array being non-zero.
    else:
        c1 = (max + min) / 2
        c2 = c1
        # Case where the rotation is clockwise based on a positive "R" value.
        if (inputArr[3] > 0):
            c1 = (slope * inputArr[3]) + intercept
        # Case where the rotation is counter-clockwise based on a negative "R" value.
        else:
            c1 = (-1) * ((slope * abs(inputArr[3])) + intercept)
        output[0] = c1
        output[1] = c1
        output[2] = c1
        output[3] = c1
        output[4] = c2
        output[5] = c2
    return output

if __name__ == '__main__':
    pi = pigpio.pi()

    for input1 in inputs:
        outputVal = control(input1)
        pi.set_servo_pulsewidth(pin, outputVal[0])
        pi.set_servo_pulsewidth(pin2, outputVal[1])
        pi.set_servo_pulsewidth(pin3, outputVal[2])
        pi.set_servo_pulsewidth(pin4, outputVal[3])
        pi.set_servo_pulsewidth(pin5, outputVal[4])
        pi.set_servo_pulsewidth(pin6, outputVal[5])
        print(outputVal)
        time.sleep(1)

    pi.stop()

# Test Code. 
# [0, 0, 0, 0]
# Connects with the raspberry pi device.
# pi = pigpio.pi()
# # Sets the signal pin value on the raspberry pi of each motor.
# pinA = 16
# pinB = 12
# pinC = 26
# pinD = 19
# pinE = 21
# pinF = 20

# output = [0, 0, 0, 0, 0, 0]

# Process the control values for each motor.