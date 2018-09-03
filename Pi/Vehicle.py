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
  def __init__(self, sensorConversionThread):
    '''
    Initializes the basics of the vehicle
    @param sensorConversionThread - the conversion thread
    '''
    self.sensorConversionThread = sensorConversionThread
    self.velocityPID = PID(Constants.VELOCITY_PID_P, Constants.VELOCITY_PID_I, Constants.VELOCITY_PID_D, Constants.VELOCITY_PID_WINDUP)
    self.steeringAnglePID = PID(Constants.STEERING_ANGLE_PID_P, Constants.STEERING_ANGLE_PID_I, Constants.STEERING_ANGLE_PID_D, Constants.STEERING_ANGLE_PID_WINDUP)
    # TODO: Wall Follow PID
    self.tabs = 0

  #-------------------------------------------------------------------------------
  def update(self, velocityGoal, steeringAngleGoal):
    '''
    Updates the vehicle's state
    @param velocityGoal - the velocity we are trying to achieve
    @param steeringAngleGoal - the steering angle we are trying to achieve
    '''
    # Set PID Goals
    self.velocityPID.setGoal(velocity)
    self.steeringAnglePID.setGoal(steeringAngle)
    # Set current vehicle measurements
    # TODO: Test. Threading issues
    vehicleVelocity = self.sensorConversionThread.velocity
    self.velocityPID.setMeasurement(vehicleVelocity)

    # TODO: Test. Threading issues
    vehicleSteeringAngle = self.sensorConversionThread.steeringAngle
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

