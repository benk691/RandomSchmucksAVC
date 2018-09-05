#!/usr/bin/python3.5
import time
import RPi.GPIO as GPIO
import Constants
from gpiozero import Button
from ControlPlanner import ControlPlanner
from CourseMap import CourseMap
from WallMap import WallMap
from DataConsumerThread import DataConsumerThread
from SensorConversion import SensorConversion
from Vehicle import Vehicle

#-------------------------------------------------------------------------------
def main():
  '''
  Main program
  '''
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
  # Setup start button
  #print("Waiting for start")
  #startButton = Button(Constants.START_BUTTON_PIN)
  #startButton.wait_for_press()
  #print("Started")
  # Start button has been pressed, so continue setup
  #courseMap = CourseMap()
  courseMap = WallMap()
  # Construct Threads
  # TODO: Untie threads
  dataConsumerThread = DataConsumerThread(daemon=True)
  sensorConversionThread = SensorConversion(daemon=True, dataConsumerThread=dataConsumerThread)
  controlPlannerThread = ControlPlanner(daemon=True, courseMap=courseMap, sensorConversionThread=sensorConversionThread)
  vehicle = Vehicle(sensorConversionThread)
  # Register Subscribers
  controlPlannerThread.register(vehicle, vehicle.update)
  # Start threads
  dataConsumerThread.start()
  sensorConversionThread.start()
  controlPlannerThread.start()

  return [dataConsumerThread, sensorConversionThread, controlPlannerThread], vehicle

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

