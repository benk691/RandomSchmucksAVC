#!/usr/bin/python3.5
import sys
import time
import math
import Constants
from threading import Thread
from argparse import ArgumentParser
from gpiozero import Button, DistanceSensor
from IIRFilter import IIRFilter
from PID import PID
from CourseMap import CourseMap
from ControlPlanner import ControlPlanner
import Adafruit_ADS1x15
from multiprocessing import Process, Manager
import Adafruit_PCA9685
from Adafruit_BNO055 import BNO055

# Global for calibration threads
calibrating = True

#-------------------------------------------------------------------------------
def main():
  '''
  Main program
  '''
  try:
    adc, imu, pwm, leftDistSensor, rightDistSensor = setup()

    currentTime = time.time()
    startTime = currentTime
    steerPotTime = currentTime + 1.0 / (Constants.STEERING_POT_FS + 1.0)
    distTime = currentTime + 1.0 / (Constants.DIST_FS + 1.0)
    vpidTime  = currentTime + 1.0 / (Constants.VPID_FS + 1.0)
    spidTime = currentTime + 1.0 / (Constants.SPID_FS + 1.0)
    controlTime = currentTime + 1.0 / (Constants.CONT_FS + 1.0)
    magTime = currentTime + 1.0 / (Constants.MAG_FS + 1.0)
    
    tachCount = 0
    steerCount = 0
    distCount = 0
    vpidCount = 0
    spidCount = 0
    controlCount = 0
    magCount = 0

    tachCountTime = 0
    steerCountTime = 0
    distCountTime = 0
    vpidCountTime = 0
    spidCountTime = 0
    controlCountTime = 0
    magCountTime = 0

    prevVpidTime = 0.0
    prevVpidTachCount = 0.0

    rightHigh = 0
    leftHigh = 0

    totalRightStripCount = 0.0
    totalLeftStripCount = 0.0

    joinTime = currentTime + 10.0
    joinCount = 0
    joinCountTime = 0.0

    heading = 0.0
    roll = 0.0
    pitch = 0.0

    leftDist = 0.0
    rightDist = 0.0
    leftDistIirFilter = IIRFilter(Constants.DIST_IIR_FILTER_A)
    rightDistIirFilter = IIRFilter(Constants.DIST_IIR_FILTER_A)

    steeringPotValue = 0.0
    steerIirFilter = IIRFilter(Constants.STEERING_IIR_FILTER_A)

    velocityPID = PID(Constants.VELOCITY_PID_P, Constants.VELOCITY_PID_I, Constants.VELOCITY_PID_D, Constants.VELOCITY_PID_WINDUP)
    steeringAnglePID = PID(Constants.STEERING_ANGLE_PID_P, Constants.STEERING_ANGLE_PID_I, Constants.STEERING_ANGLE_PID_D, Constants.STEERING_ANGLE_PID_WINDUP)

    velocity = 0.0
    steeringAngle = 0.0

    velocityGoal = 0.0
    steeringGoal = 0.0

    velocityPID.setGoal(velocityGoal)
    steeringAnglePID.setGoal(steeringGoal)

    courseMap = CourseMap()
    controlPlanner = ControlPlanner(courseMap)
    
    jobs = []

    manager = Manager()
    returnDict = manager.dict()

    #while currentTime < (startTime + 30):
    while True:
      currentTime = time.time()

      # Tachometers
      rightTachValue = adc.read_adc(Constants.ADC_RIGHT_WHEEL_CHNL, gain=Constants.ADC_GAIN, data_rate=Constants.ADC_DATA_RATE)

      leftTachValue = adc.read_adc(Constants.ADC_LEFT_WHEEL_CHNL, gain=Constants.ADC_GAIN, data_rate=Constants.ADC_DATA_RATE)

      if rightHigh == 0 and rightTachValue > Constants.TACH_RIGHT_THRESHOLD_HIGH:
        totalRightStripCount += 0.5
        rightHigh = 1

      if rightHigh == 1 and rightTachValue < Constants.TACH_RIGHT_THRESHOLD_LOW:
        totalRightStripCount += 0.5
        rightHigh = 0

      if leftHigh == 0 and leftTachValue > Constants.TACH_LEFT_THRESHOLD_HIGH:
        totalLeftStripCount += 0.5
        leftHigh = 1

      if leftHigh == 1 and leftTachValue < Constants.TACH_LEFT_THRESHOLD_LOW:
        totalLeftStripCount += 0.5
        leftHigh = 0

      tachCount += 1
      tachCountTime = time.time() - currentTime
  
      # Thread Join
      if currentTime >= joinTime:
        for proc in jobs:
          proc.join()

        if returnDict['velocityGoal'] != -1.0:
          print("Velocity New Goal = {0}".format(returnDict['velocityGoal']))
          velocityPID.setGoal(returnDict['velocityGoal'])
        if returnDict['steeringGoal'] != -1.0:
          print("Steering New Goal = {0}".format(returnDict['steeringGoal']))
          steeringAnglePID.setGoal(returnDict['steeringGoal'])
        #print(controlPlanner)
        jobs = []
        joinCount += 1
        joinCountTime = time.time() - currentTime
        joinTime += 10.0

      # Control Step (Particle Filter)
      if currentTime >= controlTime:
        totalStripCount = (totalLeftStripCount + totalRightStripCount) / 2.0
        returnDict['velocityGoal'] = -1.0
        returnDict['steeringGoal'] = -1.0
        p = Process(target=controlPlanner.control, args=(totalStripCount, heading, leftDist, rightDist, steeringAngle, returnDict))
        jobs.append(p)
        p.start()
        joinTime = currentTime + 0.3
        #velocityGoal, steeringGoal = controlPlanner.control(totalStripCount, heading, leftDist, rightDist, steeringAngle)
        if returnDict['velocityGoal'] != -1.0:
          print("Velocity New Goal = {0}".format(returnDict['velocityGoal']))
          velocityPID.setGoal(returnDict['velocityGoal'])
        if returnDict['steeringGoal'] != -1.0:
          print("Steering New Goal = {0}".format(returnDict['steeringGoal']))
          steeringAnglePID.setGoal(returnDict['steeringGoal'])
        #velocityPID.setGoal(velocityGoal)
        #steeringAnglePID.setGoal(steeringAngle)
        controlCount += 1
        controlCountTime = time.time() - currentTime
        controlTime += 1.0 / Constants.CONT_FS
        continue

      # Velocity PID
      if currentTime >= vpidTime:
        velocity = ((totalRightStripCount + totalLeftStripCount - prevVpidTachCount) / (2.0 * (currentTime - prevVpidTime))) * Constants.STRIP_COUNT_TO_METERS
        velocityPID.setMeasurement(velocity)
        velocityDuration = 1.0 - (max(min(velocityPID.control(), 0.5), -0.5) + 0.5)
        controlChnl(pwm, Constants.PWM_DRIVE_CHNL, velocityDuration)
        prevVpidTime = currentTime
        prevVpidTachCount = totalRightStripCount + totalLeftStripCount
        vpidCount += 1
        vpidCountTime = time.time() - currentTime
        vpidTime += 1.0 / Constants.VPID_FS
        continue

      # IMU
      if currentTime >= magTime:
        heading, roll, pitch = imu.read_euler()
        heading = math.radians(heading)
        magCount += 1
        magCountTime = time.time() - currentTime
        magTime += 1.0 / Constants.MAG_FS
        continue

      # Distance Sensors
      if currentTime >= distTime:
        leftDist = leftDistIirFilter.filter(leftDistSensor.distance)
        rightDist = rightDistIirFilter.filter(rightDistSensor.distance)
        distCount += 1
        distCountTime = time.time() - currentTime
        distTime += 1.0 / Constants.DIST_FS
        continue

      # Steering PID
      if currentTime >= spidTime:
        steeringAnglePID.setMeasurement(steeringAngle)
        steeringDuration = max(min(steeringAnglePID.control(), 0.5), -0.5) + 0.5
        controlChnl(pwm, Constants.PWM_TURN_CHNL, steeringDuration)
        spidCount += 1
        spidCountTime = time.time() - currentTime
        spidTime += 1.0 / Constants.SPID_FS
        continue

      # Steering Sensor
      if currentTime >= steerPotTime:
        steeringPotValue = adc.read_adc(Constants.ADC_POT_CHNL, gain=Constants.ADC_GAIN, data_rate=Constants.ADC_DATA_RATE) 
        steeringPotValue = steerIirFilter.filter(steeringPotValue)
        steeringAngle = math.radians(Constants.STEERING_CONV_SLOPE * steeringPotValue + Constants.STEERING_Y_INTERCEPT)
        steerCount += 1
        steerCountTime = time.time() - currentTime
        steerPotTime += 1.0 / Constants.STEERING_POT_FS
        continue

    print("tachCount = {0}".format(tachCount))
    print("steerCount = {0}".format(steerCount))
    print("distCount = {0}".format(distCount))
    print("vpidCount = {0}".format(vpidCount))
    print("spidCount = {0}".format(spidCount))
    print("magCount = {0}".format(magCount))
    print("controlCount = {0}".format(controlCount))
    print("tachCountTime = {0}".format(tachCountTime))
    print("steerCountTime = {0}".format(steerCountTime))
    print("distCountTime = {0}".format(distCountTime))
    print("vpidCountTime = {0}".format(vpidCountTime))
    print("spidCountTime = {0}".format(spidCountTime))
    print("magCountTime = {0}".format(magCountTime))
    print("controlCountTime = {0}".format(controlCountTime))
    print("joinTime = {0}".format(joinTime))
    print("joinCount = {0}".format(joinCount))
    print("joinCountTime = {0}".format(joinCountTime))
    print("totalRightStripCount = {0}".format(totalRightStripCount))
    print("totalLeftStripCount = {0}".format(totalLeftStripCount))
    print("heading = {0}".format(heading))
    print("roll = {0}".format(roll))
    print("pitch = {0}".format(pitch))
    print("leftDist = {0}".format(leftDist))
    print("rightDist = {0}".format(rightDist))
    print("steeringAngle = {0}".format(steeringAngle))
    print("steeringPotValue = {0}".format(steeringPotValue))
    print("velocityPID = {0}".format(velocityPID))
    print("steeringAnglePID = {0}".format(steeringAnglePID))

    for j in jobs:
      j.join()

  finally:
    pass

#-------------------------------------------------------------------------------
def setup():
  '''
  Setup the Raspberry Pi to run
  @return list of threads
  @return vehicle object
  '''
  skipStart, calibrate = parseArgs()
  if not skipStart:
    print("Waiting for start")
    startButton = Button(Constants.START_BUTTON_PIN)
    startButton.wait_for_press()
    print("Started")
    # Start button has been pressed, so continue setup
  adc = Adafruit_ADS1x15.ADS1115()
  imu = BNO055.BNO055()
  pwm = Adafruit_PCA9685.PCA9685()
  pwm.set_pwm_freq(Constants.PWM_SENSOR_FREQ)
  if not imu.begin():
    raise RuntimeError('Failed to initialize IMU! Is the sensor connected?')
  # Distance sensors
  leftDistSensor = DistanceSensor(echo=Constants.DIST_LEFT_ECHO_PIN, trigger=Constants.DIST_LEFT_TRIGGER_PIN, max_distance=Constants.DIST_MAX_DISTANCE, queue_len=Constants.DIST_QUEUE_LENGTH)
  rightDistSensor = DistanceSensor(echo=Constants.DIST_RIGHT_ECHO_PIN, trigger=Constants.DIST_RIGHT_TRIGGER_PIN, max_distance=Constants.DIST_MAX_DISTANCE, queue_len=Constants.DIST_QUEUE_LENGTH)

  performIMUCalibration(calibrate, imu)

  heading, roll, pitch = imu.read_euler()
  Constants.MAP_HEADING_OFFSET = math.radians(Constants.HEADING_WRAP_AROUND - heading)

  print("Heading Offset = {0} deg".format(math.degrees(Constants.MAP_HEADING_OFFSET)))

  return adc, imu, pwm, leftDistSensor, rightDistSensor

#-------------------------------------------------------------------------------
def parseArgs():
  parser = ArgumentParser()
  parser.add_argument('-s', '--skip-start', dest='skipStart', action='store_true', help='If this flag is specified, do not wait for the start button')
  parser.add_argument('-c', '--calibrate', dest='calibrate', action='store_true', help='If this flag is specified, calibrate the IMU settings, otherwise read the old IMU settings.')
  args = parser.parse_args()
  return args.skipStart, args.calibrate

#-------------------------------------------------------------------------------
def performIMUCalibration(calibrate, imu):
  '''
  Performs IMU calibration
  @param calibrate - flag indicating we need to calibrate from scratch or not
  @param imu - IMU
  '''
  if calibrate:
    # Spawn threads so we can read the IMU values
    calibrateThread = Thread(target=calibrateIMU, args=(dc,))
    doneCalibrateThread = Thread(target=doneCalibrateIMU)

    calibrateThread.start()
    doneCalibrateThread.start()

    while calibrating:
      time.sleep(1)

    calibrateThread.join(timeout=5)
    doneCalibrateThread.join(timeout=5)

    # Save calibration data
    calibData = imu.get_calibration()
    print("calibData = {0}".format(calibData))
    imu.set_calibration(calibData)
    with open(Constants.CALIB_SETTINGS_FILE, 'w') as calibFile:
      calibFile.write(str(calibData))
    input("Press any key once the vehicle is aligned with its desired heading. ")

  else:
    with open(Constants.CALIB_SETTINGS_FILE, 'r') as calibFile:
      calibration = [ int(c) for c in calibFile.readline().strip().strip('[]').split(', ') ]
      imu.set_calibration(calibration)
      print("Calibrated IMU: {0}".format(calibration))
      

#-------------------------------------------------------------------------------
def calibrateIMU(imu):
  '''
  Prints the IMUs current readings
  @param dc - the data consumer thread
  '''
  while calibrating:
    pass
    #print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(dc.sensors.heading, dc.sensors.roll, dc.sensors.pitch, dc.sensors.sysCal, dc.sensors.gyroCal, dc.sensors.accelCal, dc.sensors.magCal))
    ##print("RD: {0}, LD = {1}".format(dc.sensors.rightDistance, dc.sensors.leftDistance))
    #time.sleep(1)

#-------------------------------------------------------------------------------
def doneCalibrateIMU():
  '''
  Waits for input from the user to indicate that calibration of the IMU is done
  '''
  global calibrating
  while calibrating:
    a = input("Press q once you are done with calibration. ")
    calibrating = a.lower() == 'q'

#-------------------------------------------------------------------------------
def controlChnl(pwm, chnl, pulse):
    if pulse > 0.97:
      pulse = 0.97
    if pulse < 0:
      pulse = 0
    pwm.set_pwm(chnl, 0, int(pulse / Constants.PWM_SENSOR_FREQ * 10000000.0 * 5.0 / 6.0))

#-------------------------------------------------------------------------------
def shutdown(threads):
  '''
  Shutdown the threads running
  @param threads - list of threads
  '''
  for t in threads:
    t.shutdown()

#-------------------------------------------------------------------------------
if __name__ == '__main__':
  main()

