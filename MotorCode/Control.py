import pigpio
import curses
import time

# Placeholder values giving a range for use in roation.
min = 500
max = 2500
slope = (max - min) / 2
intercept = max - slope * (1)

# Allows the control of motors given an input array and an output array.
def control(inputArr, output):
    # Checks if the given input array is valid and shows an error message if not.
    for item in inputArr:
        if item < -1 or item > 1:
            print("Error")
    # Case where there is no rotation based on the "R" value of the input array being zero.
    if (inputArr[3] == 0):
        output[4] = (slope * inputArr[2]) + intercept
        output[5] = (slope * inputArr[2]) + intercept
        c1 = 0.5 * (inputArr[0] + inputArr[1])
        c2 = 0.5 * (((-1) * inputArr[0]) + inputArr[1])
        output[0] = c1
        output[1] = c2
        output[2] = c1
        output[3] = c2
    # Case where there is rotation based on the "R" value of the input array being non-zero.
    else:
        c1 = 0
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


# Test Code. 
control([1, 1, 1, 0])
control([])

# Connects with the raspberry pi device.
pi = pigpio.pi()
# Sets the signal pin value on the raspberry pi of each motor.
pinA = 16
pinB = 12
pinC = 26
pinD = 19
pinE = 21
pinF = 20

output = [0, 0, 0, 0, 0, 0]

# Process the control values for each motor.