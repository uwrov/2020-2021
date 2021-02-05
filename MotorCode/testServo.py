#!/usr/bin/ python3.6
import pigpio
import curses
import time

# return user input
def getch_c(stdscr):
    # do not wait for input when calling getch
    stdscr.nodelay(1)

    # get keyboard input, returns -1 if none available
    c = stdscr.getch()
    if c != -1:
        return c


# move servo based on keyboard input
if __name__ == '__main__':
    pi = pigpio.pi()

    pw = 1500
    pw_step = 50
    pin = 21
    pin2 = 20
    pin3 = 16
    pin4 = 12
    pin5 = 26
    pin6 = 19

    array = []

    while True:
        key = curses.wrapper(getch_c)

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

        pi.set_servo_pulsewidth(pin, pw)
        pi.set_servo_pulsewidth(pin2, pw)
        pi.set_servo_pulsewidth(pin3, pw)
        pi.set_servo_pulsewidth(pin4, pw)
        pi.set_servo_pulsewidth(pin5, pw)
        pi.set_servo_pulsewidth(pin6, pw)
        time.sleep(0.0001)

    pi.stop()