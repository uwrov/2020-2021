import pigpio
import curses
import time

class CornerServo: 
    def __init__(self, x, y, z, r):
        try:
            self.x = x
            self.y = y
            self.z = z
            self.r = r
            self.pi = pigpio.pi()
        except x < -1 or y < -1 or x > 1 or y > 1 or z < -1 or z > 1: 
            print("Error")
        except: 
            print("Someother error")

    def controlEF(self):
        self.pi.set_servo_pulsewidth(21, max(500, 1500 - 50))
        time.sleep(0.0001)


    
    