import pigpio
import time
import numpy as np

# Placeholder values giving a range for use in roation.
min = 500
max = 2500
slope = (max - min) / 2
intercept = max - slope * (1)


inputs = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 0]]

# Gives the default values for the pins.
pin1 = 21
pin2 = 20
pin3 = 16
pin4 = 12
pin5 = 26
pin6 = 19

# Allows the control of motors given an input array and an output array.
def control(controlInputs):
    controlOutputs = [0, 0, 0, 0, 0, 0]
    # Checks if the given input array is valid and shows an error message if not.
    for item in controlInputs:
        if item < -1 or item > 1:
            print("Error")
    # Case where there is no rotation based on the "R" value of the input array being zero.
    if (controlInputs[3] == 0):
        controlOutputs[4] = (slope * controlInputs[2]) + intercept
        controlOutputs[5] = (slope * controlInputs[2]) + intercept
        c1 = (slope * 0.5 * (controlInputs[0] + controlInputs[1])) + intercept
        c2 = (slope * 0.5 * (((-1) * controlInputs[0]) + controlInputs[1])) + intercept
        controlOutputs[0] = c1
        controlOutputs[1] = c2
        controlOutputs[2] = c1
        controlOutputs[3] = c2
    # Case where there is rotation based on the "R" value of the input array being non-zero.
    else:
        c1 = (max + min) / 2
        c2 = c1
        # Case where the rotation is clockwise based on a positive "R" value.
        if (controlInputs[3] > 0):
            c1 = (slope * controlInputs[3]) + intercept
        # Case where the rotation is counter-clockwise based on a negative "R" value.
        else:
            c1 = (-1) * ((slope * abs(controlInputs[3])) + intercept)
        controlOutputs[0] = c1
        controlOutputs[1] = c1
        controlOutputs[2] = c1
        controlOutputs[3] = c1
        controlOutputs[4] = c2
        controlOutputs[5] = c2
    return controlOutputs

def apply_control(_input, pi):
    outputs = control(_input)
    pi.set_servo_pulsewidth(pin1, outputs[0])
    pi.set_servo_pulsewidth(pin2, outputs[1])
    pi.set_servo_pulsewidth(pin3, outputs[2])
    pi.set_servo_pulsewidth(pin4, outputs[3])
    pi.set_servo_pulsewidth(pin5, outputs[4])
    pi.set_servo_pulsewidth(pin6, outputs[5])


if __name__ == '__main__':
    pi = pigpio.pi()
    
    # Operates the servo connected to each pin at a specific power.
    for input in inputs:
        outputs = control(input)
        pi.set_servo_pulsewidth(pin1, outputs[0])
        pi.set_servo_pulsewidth(pin2, outputs[1])
        pi.set_servo_pulsewidth(pin3, outputs[2])
        pi.set_servo_pulsewidth(pin4, outputs[3])
        pi.set_servo_pulsewidth(pin5, outputs[4])
        pi.set_servo_pulsewidth(pin6, outputs[5])
        print(outputs)
        time.sleep(1)

    pi.stop()

# Test Code Runthrough:
# [0, 0, 0, 0]
# Connects with the raspberry pi device. [pi = pigpio.pi()]
# # Sets the signal pin value on the raspberry pi of each motor.
# [pin1 = 16; pin2 = 12; pin3 = 26;  pin4 = 19; pin5 = 21;  pin6 = 20]
# output = [0, 0, 0, 0, 0, 0]
# Process the control values for each motor.