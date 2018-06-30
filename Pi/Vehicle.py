import time
from Constants import Constants
from Motor import Motor

class Vehicle:
  '''
  Controls the Fisher Price Vehicle
  '''
  #-------------------------------------------------------------------------------
  def __init__(self, GPIO):
    '''
    Initializes the basics of the vehicle
    @param GPIO - the current instance of the RPi.GPIO import that is being used
    '''
    self.speed = 0.0
    # variables are encapsulated inside of Motor so do not need extra variables
    #self.leftMotorFreq = 0.0
    #self.rightMotorFreq = 0.0
    #self.leftMotorDutyCycle = 0.0
    #self.rightMotorDutyCycle
    # Initial frequency = 5kHz
    self.driveMotor = Motor(GPIO, Constants.HBRIDGE_S1_DRIVE_PIN, 5000, 0)
    self.turnMotor = Motor(GPIO, Constants.HBRIDGE_S2_TURN_PIN, 5000, 0)
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
    return self.speed
  
  #-------------------------------------------------------------------------------
  def drive(self):
    '''
    Drives the vehicle forward
    '''
    for dc in range(100):
      self.driveMotor.changeDutyCycle(dc)
      time.sleep(1)

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
    desc += "{0}\tspeed = {1}\n".format(self.speed)
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

