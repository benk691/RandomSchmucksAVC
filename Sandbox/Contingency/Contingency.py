#!/usr/bin/python3.5
import board
import busio
import time
import Adafruit_PCA9685
import Adafruit_ADS1x15
import math
from gpiozero import DistanceSensor
from PID import PID
from Filter import Filter
from threading import Thread

# Constants
POT_CHNL = 0
RIGHT_WHEEL_CHNL = 1
LEFT_WHEEL_CHNL = 2
MOTOR_CHNL = 14
TURN_CHNL = 15
DATA_RATE = 860

FREQ = 60 * 2
GAIN = 1
TACH_THRESHOLD = 40;
MAX_LOOP_COUNT = 15.0

STOP = 780 
STRAIGHT = 19203
RIGHT = 13800
LEFT = 26000

DIST_LEFT_ECHO_PIN = 13
DIST_LEFT_TRIGGER_PIN = 6
# e-  13
# t-  6
DIST_RIGHT_ECHO_PIN = 19
DIST_RIGHT_TRIGGER_PIN = 26

# e-  19
# t-  26
DIST_MAX_DISTANCE = 2.0
DIST_QUEUE_LENGTH = 10

MAX_WALL_DIST = 0.7

#-------------------------------------------------------------------------------
def main():
  '''
  Hacky version of our contigency plan
  '''
  #------Variable Declarations------
  # Tachometer Specific Data
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

  # Timer
  startTime = 0;
  elapsedTime = 0;
  fractional = 0;

  # Steering / Turning Setup
  turnGoal = STRAIGHT
  steeringAvg = 0.0

  steeringPID = PID(10.0, 4.0, 1.0, 25.0)
  steeringFilter = Filter(0.9)

  steeringPID.setGoal(turnGoal)

  # Velocity Setup
  velDuration = STOP
  avgVel = 0.0
  velGoal = 0.5

  velocityPID = PID(50.0, 100.0, 0.0, 3.0)
  velocityFilter = Filter(0.9)

  velocityPID.setGoal(velGoal)

  # Distance sensors
  leftDistSensor = DistanceSensor(echo=DIST_LEFT_ECHO_PIN, trigger=DIST_LEFT_TRIGGER_PIN, max_distance=DIST_QUEUE_LENGTH)
  rightDistSensor = DistanceSensor(echo=DIST_RIGHT_ECHO_PIN, trigger=DIST_RIGHT_TRIGGER_PIN, max_distance=DIST_QUEUE_LENGTH)

  # Line Following Setup
  wallFollowPID = PID(1000.0, 0.0, 0.0, 0.0)
  wallFollowPID.setGoal(MAX_WALL_DIST)

  #---------------------------------

  # Setup bus
  pwm = Adafruit_PCA9685.PCA9685()
  adc = Adafruit_ADS1x15.ADS1115()
  
  try:
    pwm.set_pwm_freq(FREQ)
    
    while True:
      if (loopCount % 1 == 0):
        prevRightTachValue = rightTachValue;
        prevLeftTachValue = leftTachValue;

      steeringPotValue = adc.read_adc(POT_CHNL, gain=GAIN, data_rate=DATA_RATE) 
      rightTachValue = adc.read_adc(RIGHT_WHEEL_CHNL, gain=GAIN, data_rate=DATA_RATE)
      leftTachValue = adc.read_adc(LEFT_WHEEL_CHNL, gain=GAIN, data_rate=DATA_RATE)

      wallFollowPID.setCurrentMeasurement(rightDistSensor.distance)
      turnGoal = (- wallFollowPID.control() + STRAIGHT)

      steeringFilter.recvMeasurement(steeringPotValue)
      steeringPotValue = steeringFilter.filter()

      steeringPID.setCurrentMeasurement(steeringPotValue / 1000.0)
      steeringPID.setGoal(turnGoal / 1000.0)

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

      # Steering
      steeringDuration = steeringPID.control()
      if steeringDuration > 0:
        #steeringDuration = 670
        # Dead Band
        steeringDuration = 675 - steeringDuration
      elif steeringDuration < 0:
        #steeringDuration = 890
        steeringDuration = 875 - steeringDuration
      # Max Zone
      if steeringDuration > 930:
        steeringDuration = 930
      if steeringDuration < 630:
        steeringDuration = 630

      controlChnl(pwm, MOTOR_CHNL, int(velDuration))
      controlChnl(pwm, TURN_CHNL, int(steeringDuration))

      # Wall follow PID
      #wallFollowPID.setCurrentMeasurement(leftDistSensor.distance)
      #turnGoal = wallFollowPID.control()

      if (loopCount >= MAX_LOOP_COUNT):
        elapsedTime = (time.time() * 1000) - startTime;
        startTime = (time.time() * 1000)
        rightVelocity = ((rightStripCount / 30.0) * 0.36 * math.pi) / (elapsedTime / 1000.0); 
        leftVelocity = ((leftStripCount / 30.0) * 0.36 * math.pi) / (elapsedTime / 1000.0);

        # Velocity
        avgVelocity = (leftVelocity + rightVelocity) / 2.0
        if velDuration > STOP:
          avgVelocity *= -1

        velocityFilter.recvMeasurement(avgVelocity)
        avgVelocity = velocityFilter.filter()
        velocityPID.setCurrentMeasurement(avgVelocity)
        velocityPID.setGoal(velGoal)
        velDuration = STOP - velocityPID.control()
        if velDuration < 550:
          velDuration = 550
        if velDuration > 1000:
          velDuration = 1000

        print("LDist: {0}".format(leftDistSensor.distance))
        print("RDist: {0}".format(rightDistSensor.distance))
        print("LV: {0}".format(leftVelocity))
        print("RV: {0}".format(rightVelocity))
        print("Avg Vel: {0}".format(avgVelocity))
        print("S: {0}".format(steeringPotValue))
        print("ET: {0}".format(elapsedTime))
        print("VD: {0}".format(int(velDuration)))
        print("VG: {0}".format(velGoal))
        print("SD: {0}".format(int(steeringDuration)))
        print("SG: {0}".format(turnGoal))
        print('Steering PID:')
        print(steeringPID)
        print('Velocity PID:')
        print(velocityPID)
        print('Wall Follow PID:')
        print(wallFollowPID)
        print('')

        loopCount = 0;
        rightStripCount = 0;
        leftStripCount = 0;
        steeringAvg = 0.0


  finally:
    ramp(pwm, MOTOR_CHNL, velDuration, STOP, 1, 5)
    ramp(pwm, TURN_CHNL, steeringDuration, STOP, 1, 5)
    controlChnl(pwm, MOTOR_CHNL, STOP)
    controlChnl(pwm, TURN_CHNL, STOP)

#-------------------------------------------------------------------------------
def controlChnl(pwm, chnl, pulse):
    pwm.set_pwm(chnl, 0, int(pulse))

#-------------------------------------------------------------------------------
def ramp(pwm, chnl, curVal, rampVal, rampTime, velInc):
  '''
  Ben's hacky way of doing a ramp function
  '''
  pulse = (rampVal - curVal) / velInc
  for i in range(0, velInc):
    curVal += pulse
    controlChnl(pwm, chnl, curVal)
    time.sleep(rampTime / velInc)


#-------------------------------------------------------------------------------
if __name__ == '__main__':
  main()
