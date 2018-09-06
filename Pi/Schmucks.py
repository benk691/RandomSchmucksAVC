#!/usr/bin/python3.5
import sys
import time
import math
import Constants
from threading import Thread
from argparse import ArgumentParser
from gpiozero import Button
from ControlPlanner import ControlPlanner
from CourseMap import CourseMap
from WallMap import WallMap
from DataConsumerThread import DataConsumerThread
from SensorConversion import SensorConversion
from Vehicle import Vehicle

# Global for calibration threads
calibrating = True

#-------------------------------------------------------------------------------
def main():
  '''
  Main program
  '''
  threads = []
  try:
    threads, vehicle = setup()

    while True:
      vehicle.drive()
      for t in threads:
        print(t)

  finally:
    shutdown(threads)


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
  #courseMap = CourseMap()
  courseMap = WallMap()
  # Construct Threads
  # TODO: Untie threads
  dataConsumerThread = DataConsumerThread(daemon=True)
  sensorConversionThread = SensorConversion(daemon=True, dataConsumerThread=dataConsumerThread)
  controlPlannerThread = ControlPlanner(daemon=True, courseMap=courseMap, sensorConversionThread=sensorConversionThread)
  vehicle = Vehicle(sensorConversionThread)
  # TODO: IMPORTANT: Send 0.5 pulse before start button
  # Register Subscribers
  controlPlannerThread.register(vehicle, vehicle.updateGoals)
  # Start threads
  dataConsumerThread.start()
  sensorConversionThread.start()
  controlPlannerThread.start()

  performIMUCalibration(calibrate, dataConsumerThread)

  Constants.MAP_HEADING_OFFSET = math.radians(Constants.HEADING_WRAP_AROUND - dataConsumerThread.sensors.heading)

  print("Heading Offset = {0} deg".format(math.degrees(Constants.MAP_HEADING_OFFSET)))

  return [dataConsumerThread, sensorConversionThread, controlPlannerThread], vehicle

#-------------------------------------------------------------------------------
def parseArgs():
  parser = ArgumentParser()
  parser.add_argument('-s', '--skip-start', dest='skipStart', action='store_true', help='If this flag is specified, do not wait for the start button')
  parser.add_argument('-c', '--calibrate', dest='calibrate', action='store_true', help='If this flag is specified, calibrate the IMU settings, otherwise read the old IMU settings.')
  args = parser.parse_args()
  return args.skipStart, args.calibrate

#-------------------------------------------------------------------------------
def performIMUCalibration(calibrate, dc):
  '''
  Performs IMU calibration
  @param calibrate - flag indicating we need to calibrate from scratch or not
  @param dc - the data consumer
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
    calibData = dc.sensors.imu.get_calibration()
    print("calibData = {0}".format(calibData))
    dc.sensors.imu.set_calibration(calibData)
    with open(Constants.CALIB_SETTINGS_FILE, 'w') as calibFile:
      calibFile.write(str(calibData))
    input("Press any key once the vehicle is aligned with its desired heading. ")

  else:
    # TODO: read from settings file to set IMU
    pass

#-------------------------------------------------------------------------------
def calibrateIMU(dc):
  '''
  Prints the IMUs current readings
  @param dc - the data consumer thread
  '''
  while calibrating:
    print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(dc.sensors.heading, dc.sensors.roll, dc.sensors.pitch, dc.sensors.sysCal, dc.sensors.gyroCal, dc.sensors.accelCal, dc.sensors.magCal))
    #print("RD: {0}, LD = {1}".format(dc.sensors.rightDistance, dc.sensors.leftDistance))
    time.sleep(1)

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

