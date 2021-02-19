import pigpio
import curses
import time

def control(inputArr, output):
    for item in inputArr: 
        if item < -1 or item > 1:
            print("Error")
    #checking if the fourth value is 0 or not
    if (inputArr[3] == 0):
        output[4] = (1000 * inputArr[2]) + 2500
        output[5] = (1000 * inputArr[2]) + 2500
        c1 = 0.5 * (inputArr[0] + inputArr[1])
        c2 = 0.5 * (((-1) * inputArr[0]) + inputArr[1])
        output[0] = c1
        output[1] = c2
        output[2] = c1
        output[3] = c2
    else: 
    #if not ...

#Test Code: 
control([1, 1, 1, 0])

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



    
    
    