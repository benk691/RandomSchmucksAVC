#!/usr/bin/python3.5
import time
import RPi.GPIO as GPIO
import Constants
from ControlPlanner import ControlPlanner
from CourseMap import CourseMap
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

  finally:
    shutdown(threads)
    GPIO.cleanup()

#-------------------------------------------------------------------------------
def setup():
  '''
  Setup the Raspberry Pi to run
  @return list of threads
  '''
  courseMap = CourseMap()
  # Construct Threads
  # TODO: Untie threads
  dataConsumerThread = DataConsumerThread(daemon=True)
  sensorConversionThread = SensorConversion(daemon=True, dataConsumerThread=dataConsumerThread)
  controlPlannerThread = ControlPlanner(daemon=True, courseMap=courseMap)
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
    t.shutDown()

#-------------------------------------------------------------------------------
if __name__ == '__main__':
  main()

