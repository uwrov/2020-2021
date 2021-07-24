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
    # [0.5, 0.5, 0.5, 0, 0, 0], [0, 0, 0, 0.5, 0, 0] 
    # [0.5, 0.5, 0.5, 0.5, 0, 0]
    controlOutputs = [0, 0, 0, 0, 0, 0]
    # Checks if the given input array is valid and shows an error message if not.
    for item in controlInputs:
        assert item >= -1 or item <= 1

    # flip inputs as needed
    controlInputs[0] *= -1    # X
    controlInputs[1] *= 1     # Y
    controlInputs[2] *= -1    # Z
    controlInputs[3] *= 1     # R
    
    c1 = controlInputs[3] / 2
    # c2 = -1 * c1

    # Test Cases: [1, 0.5, 0.25, 0.5, 0, 0]; [1, 0.5, 0.5, 0.25, 1, 1]; [1, 0, -0.5, 0.25, 0, 1]

    # ----------------------------------------------------------
    # 1. Finish Limit Speed Section
    # 2. Apply R (direction) as scaled be speed (magnitude)

    max_r_speed = 2500
    min_r_speed = 500
    max_l_speed = 2500
    min_l_speed = 500
    r = controlInputs[3]
    
    # TO DO: Limit on Maximum and Minimum Linear Speed
    # Determine rotational and linear, maximum and minimum, speeds.
    limit_max = (max_r_speed + (2 * max_l_speed)) / 3000
    limit_min = (min_r_speed + (2 * min_l_speed)) / 3000

    # TO DO: R mapped (-1, 0, 1)
    # Determine rotational value source.
    # Speeds some range between 1500 to 2500 witch change of 1000.
    # Use this speed range as part of the magnitude scaling component.
    r_mapped = limit_max * r

    # Scaling an R_mapped value will compensate for rotational speed.
    Ac = controlInputs[0] - controlInputs[1] + r_mapped
    Bc = controlInputs[0] + controlInputs[1] - r_mapped
    Cc = controlInputs[0] + controlInputs[1] + r_mapped
    Dc = controlInputs[0] - controlInputs[1] - r_mapped
    Ec = controlInputs[2]
    Fc = Ec

    # Rotational Changes in pipeline 0 [c2], 1[c1], 2[c2], 3[c1], 4[0], 5[0]
    
    apply_motor(controlOutputs, 0, Ac) 
    apply_motor(controlOutputs, 1, Bc) 
    apply_motor(controlOutputs, 2, Cc) 
    apply_motor(controlOutputs, 3, Dc) 
    apply_motor(controlOutputs, 4, Ec) 
    apply_motor(controlOutputs, 5, Fc) 
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
    
    inputs = [1, 0.5, 0.25, 0.5, 0, 0]
    outputs = control(inputs)
    print(outputs)
    # Operates the servo connected to each pin at a specific power.
    pi.set_servo_pulsewidth(pin1, outputs[0])
    pi.set_servo_pulsewidth(pin2, outputs[1])
    pi.set_servo_pulsewidth(pin3, outputs[2])
    pi.set_servo_pulsewidth(pin4, outputs[3])
    pi.set_servo_pulsewidth(pin5, outputs[4])
    pi.set_servo_pulsewidth(pin6, outputs[5])
    time.sleep(20)

    pi.stop()

# Test Code Runthrough:
# [0, 0, 0, 0]
# Connects with the raspberry pi device. [pi = pigpio.pi()]
# # Sets the signal pin value on the raspberry pi of each motor.
# [pin1 = 16; pin2 = 12; pin3 = 26;  pin4 = 19; pin5 = 21;  pin6 = 20]
# output = [0, 0, 0, 0, 0, 0]
# Process the control values for each motor.
