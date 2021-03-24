import curses
import pigpio
import time

# Class to allow operation of a corner servo motor.
class CornerServo: 
    # Checks whether an error occurs for the given input array.
    def __init__(self, x, y, z, r):
        try:
            self.x = x
            self.y = y
            self.z = z
            self.r = r
            self.pi = pigpio.pi()
        # Checks if the given input array is valid and shows an error message if not.
        except x < -1 or y < -1 or x > 1 or y > 1 or z < -1 or z > 1: 
            print("Error: The given input is invalid.")
        except: 
            print("Error: Non-standard error.")

    # Allows the control of a corner servo motor.
    def controlEF(self):
        self.pi.set_servo_pulsewidth(21, max(500, 1500 - 50))
        time.sleep(0.0001)