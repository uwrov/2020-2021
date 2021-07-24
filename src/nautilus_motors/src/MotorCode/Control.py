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

#pin4 = 12 #original pin
pin4 = 19
pin5 = 26

#pin6 = 19 # original pin
pin6 = 12

# scalers applied onto each motors, index 0 is motor A and index 5 is motor F
motor_scalers = [0.25, -0.25, -0.25, 0.25, 0.25, 0.25]


# controlOutputs - array of pwms to set
# idx - index of the array to set (0 -> A, 5->F)
# c - motor constant, value in [-1,1] which scales output of motor, 
#     0 is stop, -1 is full blast reverse, 1 is full blast forward
def apply_motor(controlOutputs, idx, c):
    global motor_scalers
    controlOutputs[idx] = (motor_scalers[idx] * slope * c) + intercept 

# Allows the control of motors given an input array and an output array.
def control(controlInputs):
    controlOutputs = [0, 0, 0, 0, 0, 0]
    # Checks if the given input array is valid and shows an error message if not.
    for item in controlInputs:
<<<<<<< HEAD
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
=======
        assert item >= -1 or item <= 1

    # flip inputs as needed
    controlInputs[0] *= -1      # X 
    controlInputs[1] *= 1      # Y
    controlInputs[2] *= -1      # Z
    controlInputs[3] *= 1      # R

    # Case where there is no rotation based on the "R" value of the input array being zero.
    if (controlInputs[3] == 0):
        # calculate motor constants
        Ac = 0.5 * (controlInputs[0] - controlInputs[1])
        Bc = 0.5 * (controlInputs[0] + controlInputs[1])
        Cc = 0.5 * (controlInputs[0] + controlInputs[1])
        Dc = 0.5 * (controlInputs[0] - controlInputs[1])
        Ec = controlInputs[2]
        Fc = Ec
        
        # apply constants onto motors
        apply_motor(controlOutputs, 0, Ac) 
        apply_motor(controlOutputs, 1, Bc) 
        apply_motor(controlOutputs, 2, Cc) 
        apply_motor(controlOutputs, 3, Dc) 
        apply_motor(controlOutputs, 4, Ec) 
        apply_motor(controlOutputs, 5, Fc) 
    else:
        # rotation is clockwise based on a positive "R" value.
        c1 = controlInputs[3] / 2
        c2 = -1 * c1
        
        apply_motor(controlOutputs, 0, c2)
        apply_motor(controlOutputs, 1, c1)
        apply_motor(controlOutputs, 2, c2)
        apply_motor(controlOutputs, 3, c1)

        # zero out vertical motors
        apply_motor(controlOutputs, 4, 0)
        apply_motor(controlOutputs, 5, 0)
>>>>>>> ff779a7664ab646d7135c79ebd3020ad47b2f950
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
    
<<<<<<< HEAD
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
=======
    outputs = [1500, 1500, 1500, 1500, 1500, 1500]
    #outputs = control(inputs)
    # Operates the servo connected to each pin at a specific power.
    pi.set_servo_pulsewidth(pin1, outputs[0])
    pi.set_servo_pulsewidth(pin2, outputs[1])
    pi.set_servo_pulsewidth(pin3, outputs[2])
    pi.set_servo_pulsewidth(pin4, outputs[3])
    pi.set_servo_pulsewidth(pin5, outputs[4])
    pi.set_servo_pulsewidth(pin6, outputs[5])
    print(outputs)
    time.sleep(20)
>>>>>>> ff779a7664ab646d7135c79ebd3020ad47b2f950

    pi.stop()

# Test Code Runthrough:
# [0, 0, 0, 0]
# Connects with the raspberry pi device. [pi = pigpio.pi()]
# # Sets the signal pin value on the raspberry pi of each motor.
# [pin1 = 16; pin2 = 12; pin3 = 26;  pin4 = 19; pin5 = 21;  pin6 = 20]
# output = [0, 0, 0, 0, 0, 0]
<<<<<<< HEAD
# Process the control values for each motor.
=======
# Process the control values for each motor.
>>>>>>> ff779a7664ab646d7135c79ebd3020ad47b2f950
