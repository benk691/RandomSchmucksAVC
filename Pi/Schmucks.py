#!/usr/bin/python3
import RPi.GPIO as GPIO
from Vehicle import Vehicle

#-------------------------------------------------------------------------------
def main():
  '''
  Main program
  '''

  try:
    vehicle = setup()
    while True:
      vehicle.drive()
  finally:
    GPIO.cleanup()
  #try:
  #  vehicle = setup()
  #  while True:
  #    vehicle.drive()
  #except Exception as e:
  #  print(e)
  #finally:
  #  GPIO.cleanup()

#-------------------------------------------------------------------------------
def setup():
  '''
  Setup the Raspberry Pi to run
  @return the vehicle object
  '''
  GPIO.setwarnings(False)
  GPIO.setmode(GPIO.BOARD)

  vehicle = Vehicle(GPIO)
  return vehicle

#-------------------------------------------------------------------------------
if __name__ == '__main__':
  main()

