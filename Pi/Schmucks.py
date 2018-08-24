#!/usr/bin/python3.5
import time
import RPi.GPIO as GPIO
import Constants
from Vehicle import Vehicle
from DataConsumerThread import DataConsumerThread

#-------------------------------------------------------------------------------
def main():
  '''
  Main program
  '''
  try:
    dataConsumerThread = DataConsumerThread(daemon=True)
    dataConsumerThread.start()
    while True:
      print(str(dataConsumerThread.sensors))
      time.sleep(1)
      print()
      
  finally:
    dataConsumerThread.shutdown()
    GPIO.cleanup()

#-------------------------------------------------------------------------------
def setup():
  '''
  Setup the Raspberry Pi to run
  '''
  pass

#-------------------------------------------------------------------------------
if __name__ == '__main__':
  main()

