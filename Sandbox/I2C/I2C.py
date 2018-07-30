#!/usr/bin/python3.5
import board
import busio
import time
import Adafruit_PCA9685
import Adafruit_ADS1x15
import math
from threading import Thread

# I2C Device Addresses
I2C_BUS_ADDR = 0x1 # 0x1 Indicates /dev/i2c-1
PWM_ADDR = 0x40 # I2C Adress for Pulse Width Modulation (PWM) Module
ADC_ADDR = 0x48 # Analog Digital Converter (ADC) Module
IMU_ADDR = 0x28 # Inertial Measurement Unit (IMU) Module

STOP = 775 
GAIN = 1

# Tachometer Specific Data
TACH_THRESHOLD = 40;
rightTachValue = 0;
leftTachValue = 0;
prevRightTachValue = 0;
prevLeftTachValue = 0;
rightVelocity = 0;
rightStripCount = 0;
leftVelocity = 0;
leftStripCount = 0;
loopCount = 0;

leftHigh = 0;
leftThresholdHigh = 8500;
leftThresholdLow = 6500;

rightHigh = 0;
rightThresholdHigh = 10000;
rightThresholdLow = 8000;

steeringPotValue = 0

#SLEEP_TIME = 3 * 10e-4;
SLEEP_TIME = 0.001 / 10.0
#SLEEP_TIME = 1

# Timer
startTime = 0;
elapsedTime = 0;
fractional = 0;

velGoal = 0.0
turnGoal = 0.0

#-------------------------------------------------------------------------------
def main():
  '''
  Main program to test I2C communication on the I2C
  '''
  global SLEEP_TIME
  global TACH_THRESHOLD 
  global rightTachValue
  global leftTachValue
  global prevRightTachValue
  global prevLeftTachValue
  global rightVelocity
  global rightStripCount
  global leftVelocity
  global leftStripCount
  global loopCount
  global leftHigh
  global leftThresholdHigh
  global leftThresholdLow
  global rightHigh
  global rightThresholdHigh
  global rightThresholdLow
  global steeringPotValue
  global startTime
  global elapsedTime
  global fractional

  # Setup bus
  pwm = Adafruit_PCA9685.PCA9685()
  adc = Adafruit_ADS1x15.ADS1115()

  # Thread
  goalThread = Thread(target=recvGoal)
  goalThread.start()

  try:
    values = [0] * 4
    # motor
    motorChnl = 14
    freq=60 * 2
    pwm.set_pwm_freq(freq)
    print("Freq = {0}".format(freq))

    rightWheelChnl = 1
    leftWheelChnl = 2
    potChnl = 0
    # Data Rate of ADC
    DATA_RATE = 860

    pulseDuration = STOP
    error = 0.0
    avgVelocity = 0.0

    while True:
      if (loopCount % 1 == 0):
        prevRightTachValue = rightTachValue;
        prevLeftTachValue = leftTachValue;

      steeringPotValue = adc.read_adc(potChnl, gain=GAIN, data_rate=DATA_RATE) 
      rightTachValue = adc.read_adc(rightWheelChnl, gain=GAIN, data_rate=DATA_RATE)
      leftTachValue = adc.read_adc(leftWheelChnl, gain=GAIN, data_rate=DATA_RATE)

      #print("LT: {0}".format(rightTachValue))
      #print("RT: {0}".format(leftTachValue))
      #print("ST: {0}".format(steeringPotValue))

      controlChnl(pwm, motorChnl, int(pulseDuration))

      if (rightHigh == 0 and  rightTachValue > rightThresholdHigh):
        rightStripCount += 0.5;
        rightHigh = 1;

      if (rightHigh == 1 and  rightTachValue < rightThresholdHigh):
        rightStripCount += 0.5;
        rightHigh = 0;

      if (leftHigh == 0 and  leftTachValue > leftThresholdHigh):
        leftStripCount += 0.5;
        leftHigh = 1;

      if (leftHigh == 1 and  leftTachValue < leftThresholdHigh):
        leftStripCount += 0.5;
        leftHigh = 0;
      
      loopCount += 1
      #print(loopCount)

      if (loopCount >= 50):
        elapsedTime = (time.time() * 1000) - startTime;
        startTime = (time.time() * 1000)
        rightVelocity = ((rightStripCount / 30.0) * 0.36 * math.pi) / (elapsedTime / 1000.0); 
        leftVelocity = ((leftStripCount / 30.0) * 0.36 * math.pi) / (elapsedTime / 1000.0);

        avgVelocity = (leftVelocity + rightVelocity) / 2.0
        error = velGoal - avgVelocity
        pulseDuration = pulseDuration - error * 50.0

        print("LV: {0}".format(leftVelocity))
        print("RV: {0}".format(rightVelocity))
        print("S: {0}".format(steeringPotValue))
        print("ET: {0}".format(elapsedTime))
        print("PD: {0}".format(int(pulseDuration)))
        print("VG: {0}".format(velGoal))
        print("error: {0}".format(error))
        print('')

        loopCount = 0;
        rightStripCount = 0;
        leftStripCount = 0;

        #time.sleep(SLEEP_TIME);   

    #for pulse in [ i for i in range(450, 1000) if i % 5 == 0 ]:
    #controlChnl(pwm, motorChnl, STOP)

    # tach
    # Read all ADC values
    #for pulse in [ i for i in range(450, 1000) if i % 5 == 0 ]:
    #  controlChnl(pwm, motorChnl, pulse)
    #  for i in range(4):
    #    values[i] = adc.read_adc(i, gain=GAIN)
    #    print("values[{0}] = {1}".format(i, values[i]))
    #  time.sleep(1)

    # Reads ADC values
    #while True:
    #  for i in range(4):
    #    values[i] = adc.read_adc(i, gain=GAIN)
    #    print("values[{0}] = {1}".format(i, values[i]))
    #  print('')
    #  time.sleep(1)
    
  finally:
    controlChnl(pwm, motorChnl, STOP)

#def getTime():
#  return time.time()
#
#def getElapseTime(sTime, eTime):
#  print("Elapsed: {0}".format(sTime)

#-------------------------------------------------------------------------------
def controlChnl(pwm, chnl, pulse):
    pwm.set_pwm(chnl, 0, pulse)
    #print("Chnl {0}: Pulse: {1}".format(chnl, pulse))

#-------------------------------------------------------------------------------
def recvGoal():
  # Modify so this is on demand
  global velGoal
  global turnGoal
  while True:
    velGoal = float(input("Enter velocity goal: "))
    #turnGoal = float(input("Enter turn goal: "))

#-------------------------------------------------------------------------------
if __name__ == '__main__':
  main()
