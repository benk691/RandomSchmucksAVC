import time
import re
import Constants
from PID import PID

# TODO: Subscribe to the control planner
class Vehicle:
  '''
  Controls the Fisher Price Vehicle
  '''

  #-------------------------------------------------------------------------------
  def __init__(self):
    '''
    Initializes the basics of the vehicle
    '''
    self.velocityPID = PID(Constants.VELOCITY_PID_P, Constants.VELOCITY_PID_I, Constants.VELOCITY_PID_D, Constants.VELOCITY_PID_WINDUP)
    self.steeringAnglePID = PID(Constants.STEERING_ANGLE_PID_P, Constants.STEERING_ANGLE_PID_I, Constants.STEERING_ANGLE_PID_D, Constants.STEERING_ANGLE_PID_WINDUP)
    # TODO: Wall Follow PID
    self.tabs = 0

  #-------------------------------------------------------------------------------
  def setVelocity(self, velocity):
    '''
    Sets the velocity goal of the PID control
    @param velocity - the goal velocity
    '''
    self.velocityPID.setGoal(velocity)
    # TODO: request current vehicle measurement
    vehicleVelocity = 0.0
    self.velocityPID.setMeasurement(vehicleVelocity)

  #-------------------------------------------------------------------------------
  def setTurnAngle(self, steeringAngle):
    '''
    Sets the turn angle goal of the PID control
    @param steeringAngle - the goal turn angle
    '''
    self.steeringAnglePID.setGoal(steeringAngle)
    # TODO: request current vehicle measurement
    vehicleSteeringAngle = 0.0
    self.velocityPID.setMeasurement(vehicleSteeringAngle)

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
    desc += "{0}\tvelocityPID = {1}\n".format('\t' * self.tabs, self.velocityPID)
    desc += "{0}\tsteeringAnglePID = {1}\n".format('\t' * self.tabs, self.steeringAnglePID)
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

