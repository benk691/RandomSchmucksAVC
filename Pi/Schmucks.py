#!/usr/bin/python3.5
import time
import RPi.GPIO as GPIO
from Constants import Constants
from Vehicle import Vehicle
from DataConsumer import DataConsumer

#-------------------------------------------------------------------------------
def main():
  '''
  Main program
  '''

  try:
    vehicle = setup()
    #dataConsumer = DataConsumer(ser)
    #dataConsumer.start()
    while True:
      pass
      #vehicle.getSpeed()
      #print(vehicle)
      #time.sleep(1)
      
  finally:
    #dataConsumer.shutdown()
    GPIO.cleanup()

#-------------------------------------------------------------------------------
def setup():
  '''
  Setup the Raspberry Pi to run
  @return the vehicle object
  @return the serial communication line
  '''
  GPIO.setwarnings(False)
  GPIO.setmode(GPIO.BOARD)

  vehicle = Vehicle(GPIO)
  return vehicle

#-------------------------------------------------------------------------------
if __name__ == '__main__':
  main()

