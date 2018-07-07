#!/usr/bin/python3
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
      vehicle.getSpeed()
      print(vehicle)
      time.sleep(1)
  finally:
    GPIO.cleanup()

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

