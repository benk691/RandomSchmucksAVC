#!/usr/bin/python3.5
import board
import busio
import time
import Adafruit_PCA9685

# I2C Device Addresses
I2C_BUS_ADDR = 0x1 # 0x1 Indicates /dev/i2c-1
PWM_ADDR = 0x40 # I2C Adress for Pulse Width Modulation (PWM) Module
ADC_ADDR = 0x48 # Analog Digital Converter (ADC) Module
IMU_ADDR = 0x28 # Inertial Measurement Unit (IMU) Module

STOP = 775 

#-------------------------------------------------------------------------------
def main():
  '''
  Main program to test I2C communication on the I2C
  '''
  # Setup bus
  pwm = Adafruit_PCA9685.PCA9685()

  # motor
  motorChnl = 15
  freq=60 * 2
  pwm.set_pwm_freq(freq)
  print("Freq = {0}".format(freq))
  #while True:
  for pulse in [ i for i in range(450, 1000) if i % 5 == 0 ]:
    controlChnl(pwm, motorChnl, pulse)
  
  controlChnl(pwm, motorChnl, STOP)

#-------------------------------------------------------------------------------
def controlChnl(pwm, chnl, pulse):
    pwm.set_pwm(chnl, 0, pulse)
    print("Chnl {0}: Pulse: {1}".format(chnl, pulse))
    time.sleep(0.5)

#-------------------------------------------------------------------------------
# Deleted
'''
#import smbus
#import motor
  #bus = smbus.SMBus(I2C_BUS_ADDR)
  #i2c = busio.I2C(board.SCL, board.SDA)
  #pca = Adafruit_PCA9685.PCA9685(i2c)
  #driveMotorChnl = pca.channels[0]
  #pca.frequency = 5000
'''

#-------------------------------------------------------------------------------
if __name__ == '__main__':
  main()
