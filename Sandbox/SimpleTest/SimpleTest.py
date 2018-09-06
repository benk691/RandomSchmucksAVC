#!/usr/bin/python3.5
import board
import busio
import time
import Adafruit_PCA9685
import Adafruit_ADS1x15
import math

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
  # Setup bus
  pwm = Adafruit_PCA9685.PCA9685()
  
  try:
    pwm.set_pwm_freq(FREQ)
    print("Start")
    direction = 1
    steeeringDuration = 0.5
    velocotyDuration = 0.5
    while True:
      controlChnl(pwm, TURN_CHNL, steeeringDuration)
      controlChnl(pwm, MOTOR_CHNL, velocotyDuration)
      steeeringDuration += direction * 0.05
      velocotyDuration += direction * 0.05
      if steeeringDuration >= 0.8 or velocotyDuration >= 0.8:
        direction *= -1

      if steeeringDuration <= 0.2 or velocotyDuration <= 0.2:
        direction *= -1

      print("velocotyDuration = {0}".format(velocotyDuration))
      print("steeeringDuration = {0}".format(steeeringDuration))
      time.sleep(1.0)
      
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
    pwm.set_pwm(chnl, 0, int(pulse / FREQ * 10000000.0 * 5.0 / 6.0))

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

