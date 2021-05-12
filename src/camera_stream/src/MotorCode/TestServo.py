import curses
import pigpio
import time

# Return the users input.
def getch_c(stdscr):
    # Do not wait for input when calling getch.
    stdscr.nodelay(1)

    # Gets keyboard input, returns -1 if none available.
    c = stdscr.getch()
    if c != -1:
        return c


# Moves the servo motor based on keyboard input.
if __name__ == '__main__':
    pi = pigpio.pi()

    # Gives the default values for the power and pins.
    pw = 1500
    pw_step = 50
    pin1 = 21
    pin2 = 20
    pin3 = 16
    pin4 = 12
    pin5 = 26
    pin6 = 19

    array = []

    while True:
        key = curses.wrapper(getch_c)

        # Takes the keyboard input and designates the power.
        if (key == ord('d')):
            pw = max(500, pw - pw_step)
        elif (key == ord('a')):
            pw = min(2500, pw + pw_step)
        elif (key == ord('w')):
            pw_step = min(500, pw_step + 10)
        elif (key == ord('s')):
            pw_step = max(10, pw_step - 10)
        elif (key == ord('q')):
            break
        
        # Operates the servo connected to each pin at a set power.
        pi.set_servo_pulsewidth(pin1, pw)
        pi.set_servo_pulsewidth(pin2, pw)
        pi.set_servo_pulsewidth(pin3, pw)
        pi.set_servo_pulsewidth(pin4, pw)
        pi.set_servo_pulsewidth(pin5, pw)
        pi.set_servo_pulsewidth(pin6, pw)
        time.sleep(0.0001)

    pi.stop()
