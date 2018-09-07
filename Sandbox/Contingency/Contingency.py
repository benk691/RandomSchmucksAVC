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

# Constants
POT_CHNL = 0
RIGHT_WHEEL_CHNL = 1
LEFT_WHEEL_CHNL = 2
MOTOR_CHNL = 14
TURN_CHNL = 15
DATA_RATE = 860

FREQ = 2000
GAIN = 1
TACH_THRESHOLD = 40;
MAX_LOOP_COUNT = 15.0

#STOP = 780 
STOP = 0 
STRAIGHT = 19203
RIGHT = 13800
LEFT = 26000

DIST_LEFT_ECHO_PIN = 27
DIST_LEFT_TRIGGER_PIN = 17
# e-  13
# t-  6
DIST_RIGHT_ECHO_PIN = 22
DIST_RIGHT_TRIGGER_PIN = 18

# e-  19
# t-  26
DIST_MAX_DISTANCE = 2
DIST_QUEUE_LENGTH = 10

MAX_WALL_DIST = 1.5

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

  distTraveled = 0.0
  distStartTime = 0.0
  distEndTime = 0.0

  # Timer
  startTime = 0;
  elapsedTime = 0;
  fractional = 0;

  # Steering / Turning Setup
  turnGoal = STRAIGHT
  steeringAvg = 0.0

  steeringPID = PID(1.0, 0.2, 0.0, 3.0)
  steeringFilter = Filter(0.9)

  steeringPID.setGoal(turnGoal)

  # Velocity Setup
  velDuration = STOP
  avgVelocity = 0.0
  velGoal = 0.5

  velocityPID = PID(50.0, 100.0, 0.0, 3.0)
  velocityFilter = Filter(0.9)

  velocityPID.setGoal(velGoal)

  # Distance sensors
  leftDistSensor = DistanceSensor(echo=DIST_LEFT_ECHO_PIN, trigger=DIST_LEFT_TRIGGER_PIN, max_distance=DIST_MAX_DISTANCE, queue_len=DIST_QUEUE_LENGTH)
  rightDistSensor = DistanceSensor(echo=DIST_RIGHT_ECHO_PIN, trigger=DIST_RIGHT_TRIGGER_PIN, max_distance=DIST_MAX_DISTANCE, queue_len=DIST_QUEUE_LENGTH)
  distFilter = Filter(0.8)

  leftDist = 0.0
  rightDist = 0.0

  # Line Following Setup
  wallFollowPID = PID(10000.0, 0.0, 0.0, 0.8)
  wallFollowPID.setGoal(MAX_WALL_DIST)

  #---------------------------------

  # Setup bus
  pwm = Adafruit_PCA9685.PCA9685()
  adc = Adafruit_ADS1x15.ADS1115()
  
  try:
    pwm.set_pwm_freq(FREQ)
    print("Start")
    distStartTime = time.time()
    while True:
      if (loopCount % 1 == 0):
        prevRightTachValue = rightTachValue;
        prevLeftTachValue = leftTachValue;

      steeringPotValue = adc.read_adc(POT_CHNL, gain=GAIN, data_rate=DATA_RATE) 
      rightTachValue = adc.read_adc(RIGHT_WHEEL_CHNL, gain=GAIN, data_rate=DATA_RATE)
      leftTachValue = adc.read_adc(LEFT_WHEEL_CHNL, gain=GAIN, data_rate=DATA_RATE)

      distTraveled += (avgVelocity / (time.time() - distStartTime))

      #distFilter.recvMeasurement(rightDistSensor.distance)
      #rightDist = distFilter.filter()

      #distFilter.recvMeasurement(leftDistSensor.distance)
      #leftDist = distFilter.filter()
      rightDist = rightDistSensor.distance
      leftDist = leftDistSensor.distance

      #print('Left Distance: ', leftDistSensor.distance * 100)
      #print('Right Distance: ', rightDistSensor.distance * 100)

      # Wall follow PID
      wallFollowPID.setCurrentMeasurement(rightDist)
      turnGoal = (wallFollowPID.control() + STRAIGHT)

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
      steeringDuration = - steeringPID.control() + 0.5
      '''
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
      '''
      controlChnl(pwm, MOTOR_CHNL, velDuration)
      #controlChnl(pwm, MOTOR_CHNL, 0.75)

      #print("SD: {0}".format(steeringDuration))
      controlChnl(pwm, TURN_CHNL, steeringDuration)
      #controlChnl(pwm, TURN_CHNL, 0.99)
      #controlChnl(pwm, TURN_CHNL, 930)
      #controlChnl(pwm, TURN_CHNL, 0.0)
      #controlChnl(pwm, TURN_CHNL, 780)
      #controlChnl(pwm, TURN_CHNL, velDuration)
      #controlChnl(pwm, TURN_CHNL, s)

      #print("VD: {0}".format(int(velDuration)))
      #print("SD: {0}".format(int(steeringDuration)))

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
        #if velDuration < 550:
        #  velDuration = 550
        #if velDuration > 1000:
        #  velDuration = 1000

        print('Total Distance: ', distTraveled)
        print('Left Distance: ', leftDistSensor.distance * 100)
        print('Right Distance: ', rightDistSensor.distance * 100)
        print("LDist: {0}".format(leftDist))
        print("RDist: {0}".format(rightDist))
        print("LDist: {0}".format(leftDistSensor.distance * 100))
        print("RDist: {0}".format(rightDistSensor.distance * 100))
        print("LV: {0}".format(leftVelocity))
        print("RV: {0}".format(rightVelocity))
        print("Avg Vel: {0}".format(avgVelocity))
        print("S: {0}".format(steeringPotValue))
        print("ET: {0}".format(elapsedTime))
        print("VD: {0}".format(velDuration))
        print("VG: {0}".format(velGoal))
        print("SD: {0}".format(steeringDuration))
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
    #ramp(pwm, MOTOR_CHNL, velDuration, STOP, 1, 5)
    #ramp(pwm, TURN_CHNL, steeringDuration, STOP, 1, 5)
    #controlChnl(pwm, MOTOR_CHNL, STOP)
    #controlChnl(pwm, TURN_CHNL, STOP)
    pass

#-------------------------------------------------------------------------------
def controlChnl(pwm, chnl, pulse):
    if pulse > 0.97:
      pulse = 0.97
    if pulse < 0:
      pulse = 0
    pwm.set_pwm(chnl, 0, int(pulse / Constants.PWM_SENSOR_FREQ * 10000000.0 * 5.0 / 6.0))

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

