import time
import re
from Constants import Constants
from Motor import Motor

class Vehicle:
  '''
  Controls the Fisher Price Vehicle
  '''
  # Regex to get decimal values
  decMatch = re.compile(r'\d+\.\d+')

  #-------------------------------------------------------------------------------
  def __init__(self, GPIO, serial):
    '''
    Initializes the basics of the vehicle
    @param GPIO - the current instance of the RPi.GPIO import that is being used
    @param serial - the serial communication line
    '''
    self.leftVelocity = 0
    self.rightVelocity = 0
    # Initial frequency = 5kHz
    self.driveMotor = Motor(GPIO, Constants.HBRIDGE_S1_DRIVE_PIN, 5000, 0)
    self.turnMotor = Motor(GPIO, Constants.HBRIDGE_S2_TURN_PIN, 5000, 0)
    self.serial = serial
    self.tabs = 0

  #-------------------------------------------------------------------------------
  def setSpeed(self, speed):
    '''
    Sets the speed of the vehicle
    @param speed - the input speed, this should be a decimal value that is used 
                     as an analog input into the Sabertooth H-Bridge
    '''
    self.speed = speed

  #-------------------------------------------------------------------------------
  def getSpeed(self):
    '''
    Gets the current speed of the vehicle
    @return speed of the vehicle. This is the anlog value being feed into the 
            Sabertooth H-Bridge
    '''
    velLine = self.serial.readline()
    if 'LV' in velLine and 'RV' in velLine:
      left, right = velLine.split(',')
      lValues = decMatch.findall(left)
      if lValues:
        self.leftVelocity = float(lValues[0])

      if rValues:
        self.rightVelocity = float(rValues[0])
  
  #-------------------------------------------------------------------------------
  def drive(self):
    '''
    Drives the vehicle forward
    '''
    pass
    #for dc in range(100):
    #  self.driveMotor.changeDutyCycle(dc)
    #  time.sleep(1)

  #-------------------------------------------------------------------------------
  def turn(self):
    '''
    Turns the vehicle
    '''
    for dc in range(100):
      self.turnMotor.changeDutyCycle(dc)
      time.sleep(1)

  #-------------------------------------------------------------------------------
  def setTabs(self, tabs):
    '''
    Set the number of tabs to use when printing out information
    @param tabs - the number of tabs to use in print out
    '''
    self.tabs = tabs

  #-------------------------------------------------------------------------------
  def _debugDescrition():
    '''
    Generates debugging information about the vehicle
    @return string describing debug information
    '''
    self.leftMotor.setTabs(self.tabs + 2)
    self.rightMotor.setTabs(self.tabs + 2)
    desc = "{0}Vehicle Info:\n".format('\t' * self.tabs)
    desc += "{0}\tRight Velocity = {1}\n".format(self.rightVelocity)
    desc += "{0}\tLeft Velocity = {1}\n".format(self.leftVelocity)
    desc += "{0}\tDrive Motor:\n{1}\n".format('\t' * self.tabs, self.driveMotor)
    desc += "{0}\tTurn Motor:\n{1}\n".format('\t' * self.tabs, self.turnMotor)
    return desc
  
  #-------------------------------------------------------------------------------
  def __repr__(self):
    '''
    Gets the string representation of the class
    @return string representation of the class
    '''
    return self._debugDescription()

  #-------------------------------------------------------------------------------
  def __str__(self):
    '''
    Gets the string representation of the class
    @return string representation of the class
    '''
    return self._debugDescription()

