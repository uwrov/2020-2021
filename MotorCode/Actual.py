import pigpio
import curses
import time

min = 500
max = 2500
slope = (max - min) / 2
intercept = max - slope * (1)

def control(inputArr, output):
    for item in inputArr: 
        if item < -1 or item > 1:
            print("Error")
    #checking if the fourth value is 0 or not
    if (inputArr[3] == 0):
        output[4] = (slope * inputArr[2]) + intercept
        output[5] = (slope * inputArr[2]) + intercept
        c1 = 0.5 * (inputArr[0] + inputArr[1])
        c2 = 0.5 * (((-1) * inputArr[0]) + inputArr[1])
        output[0] = c1
        output[1] = c2
        output[2] = c1
        output[3] = c2
    else: 
        c1 = 0
        if (inputArr[3] > 0):
            c1 = (slope * inputArr[3]) + intercept
        else:
            c1 = (-1) * ((slope * abs(inputArr[3])) + intercept)
        output[0] = c1
        output[1] = c1
        output[2] = c1
        output[3] = c1

    

#Test Code: 
control([1, 1, 1, 0])
control([])

#Connection with pi:
pi = pigpio.pi()

pinA = 16
pinB = 12
pinC = 26
pinD = 19
pinE = 21
pinF = 20

output = [0, 0, 0, 0, 0, 0]

#Process the input