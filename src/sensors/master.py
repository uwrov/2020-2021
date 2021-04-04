import smbus
import math
import RPi.GPIO as GPIO

from Adafruit_BMP085 import BMP085
''' MPU6050 defined functions
    read_byte(reg) :  <say what it does>
    read_word(add,reg):
    ...
'''
 
def read_byte(reg):
    return bus.read_byte_data(address, reg)
 
def read_word(add, reg):
    h = bus.read_byte_data(add, reg)
    l = bus.read_byte_data(add, reg+1)
    value = (h << 8) + l
    return value
 
def read_word_2c(add, reg):
    val = read_word(add, reg)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val
 
def dist(a,b):
    return math.sqrt((a*a)+(b*b))
 
def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)
 
def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

bus = smbus.SMBus(1) # bus = smbus.SMBus(0) fuer Revision 1 


'''
BMP 180 Variable Declaration
'''
bmp = BMP085(0x77)

#barometric data
temp = bmp.readTemperature()
pressure = bmp.readPressure()
altitude = bmp.readAltitude()

'''
Distance Sensor Data JSN
'''

def mpu6050(address) :
    
    # Register
    power_mgmt_1 = 0x6b
    power_mgmt_2 = 0x6c
    
    #print "Gyro Unit:  Gyro_x |  Gyro_y  |  Gyro_z  |  Acc_x   |  Acc_y   | Acc_z | X Rotation | Y Rotation"
    #while (stepsDone <= totalSteps):
    #=========================================
    #Gyroscope 1 reading
    #address = 0x68       # via i2cdetect
     
    # Activates first Gyroscope
    if address == 0x68:
        bus.write_byte_data(address, power_mgmt_1, 0)
    elif address == 0x69:
        bus.write_byte_data(address, power_mgmt_1, 0)
        bus.write_byte_data(address, power_mgmt_2, 0)
    #print "Gyroskop One"
    #print "------------------------"

     
    gyro_xout = read_word_2c(address, 0x43)
    gyro_yout = read_word_2c(address, 0x45)
    gyro_zout = read_word_2c(address, 0x47)

    #Properly Scaled
    Gx = gyro_xout / 131
    Gy = gyro_yout / 131
    Gz = gyro_zout / 131

     
    acc_xout = read_word_2c(address, 0x3b)
    acc_yout = read_word_2c(address, 0x3d)
    acc_zout = read_word_2c(address, 0x3f)
     
    #Properly Scaled
    Ax = acc_xout / 16384.0
    Ay = acc_yout / 16384.0
    Az = acc_zout / 16384.0
    return Gx, Gy, Gz, Ax, Ay, Az    
    
        #print "Gyro 1: ", "%.4f"%Gx, "| %.4f"%Gy, "| %.4f" %Gz, "| %.4f" %Ax, "| %.4f" %Ay, "| %.4f" %Az, "|", get_x_rotation(Ax, Ay, Az), "|", get_y_rotation(Ax, Ay, Az) 
         
        #print "X Rotation: " , get_x_rotation(Ax, Ay, Az)
        #print "Y Rotation: " , get_y_rotation(Ax, Ay, Az)
        

def main():
    """Main function of the app"""
    Gx, Gy, Gz, Ax, Ay, Az  = mpu6050(0x68)
    print "Gyro 1: ", "%.4f"%Gx, "| %.4f"%Gy, "| %.4f" %Gz, "| %.4f" %Ax, "| %.4f" %Ay, "| %.4f" %Az, "|", get_x_rotation(Ax, Ay, Az), "|", get_y_rotation(Ax, Ay, Az)     
    Gx, Gy, Gz, Ax, Ay, Az  = mpu6050(0x69)
    print "Gyro 2: ", "%.4f"%Gx, "| %.4f"%Gy, "| %.4f" %Gz, "| %.4f" %Ax, "| %.4f" %Ay, "| %.4f" %Az, "|", get_x_rotation(Ax, Ay, Az), "|", get_y_rotation(Ax, Ay, Az)     
    
    print "Temperature: %.2f C" % temp
    print "Pressure:    %.2f hPa" % (pressure / 100.0)
    print "Altitude:    %.2f" % altitude

if __name__ == '__main__':
    main()