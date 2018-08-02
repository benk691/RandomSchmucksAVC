import time
import re
from Constants import Constants

class Vehicle:
  '''
  Controls the Fisher Price Vehicle
  '''
  # Regex to get floating point values
  decMatch = re.compile(r'\d+\.\d+')

  #-------------------------------------------------------------------------------
  def __init__(self, GPIO):
    '''
    Initializes the basics of the vehicle
    @param GPIO - the current instance of the RPi.GPIO import that is being used
    '''
    self.leftVelocity = 0
    self.rightVelocity = 0
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
    @return speed of the vehicle. This is the analog value being feed into the 
            Sabertooth H-Bridge
    '''
    pass 

  #-------------------------------------------------------------------------------
  def drive(self):
    '''
    Drives the vehicle forward
    '''
    pass

  #-------------------------------------------------------------------------------
  def turn(self):
    '''
    Turns the vehicle
    '''
    pass

  #-------------------------------------------------------------------------------
  def setTabs(self, tabs):
    '''
    Set the number of tabs to use when printing out information
    @param tabs - the number of tabs to use in print out
    '''
    self.tabs = tabs

  #-------------------------------------------------------------------------------
  def _debugDescription(self):
    '''
    Generates debugging information about the vehicle
    @return string describing debug information
    '''
    desc = "{0}Vehicle Info:\n".format('\t' * self.tabs)
    desc += "{0}\tRight Velocity = {1}\n".format('\t' * self.tabs, self.rightVelocity)
    desc += "{0}\tLeft Velocity = {1}\n".format('\t' * self.tabs, self.leftVelocity)
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

