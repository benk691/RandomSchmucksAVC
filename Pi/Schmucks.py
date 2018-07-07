#!/usr/bin/python3.5
import serial
import time
import RPi.GPIO as GPIO
from Constants import Constants
from Vehicle import Vehicle

#-------------------------------------------------------------------------------
def main():
  '''
  Main program
  '''

  try:
    vehicle, ser = setup()
    while True:
      #vehicle.drive()
      print(vehicle)
      time.sleep(1)
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
  @return the serial communication line
  '''
  ser = serial.Serial(Constants.ARDUINO_SERIAL_PORT, Constants.BAUDRATE)
  ser.baudrate = Constants.BAUDRATE
  GPIO.setwarnings(False)
  GPIO.setmode(GPIO.BOARD)

  vehicle = Vehicle(GPIO, ser)
  return vehicle, ser

#-------------------------------------------------------------------------------
if __name__ == '__main__':
  main()

