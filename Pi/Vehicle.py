import time
import Constants
import Adafruit_PCA9685
from PID import PID
from fractions import gcd

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
    self.pwm = Adafruit_PCA9685.PCA9685()
    self.pwm.set_pwm_freq(Constants.PWM_SENSOR_FREQ)
    self.velocityPID = PID(Constants.VELOCITY_PID_P, Constants.VELOCITY_PID_I, Constants.VELOCITY_PID_D, Constants.VELOCITY_PID_WINDUP)
    self.steeringAnglePID = PID(Constants.STEERING_ANGLE_PID_P, Constants.STEERING_ANGLE_PID_I, Constants.STEERING_ANGLE_PID_D, Constants.STEERING_ANGLE_PID_WINDUP)
    self.velocityDuration = -0.0
    self.steeringDuration = -0.0
    self.prevTotalStripCount = 0.0
    self.totalStripCount = 0.0
    # TODO: Wall Follow PID
    self.tabs = 0
    self.totalUpdateTime = 0.0
    self.prevTime = time.time()
    self.currentTime = time.time()
    self.updateRate = gcd(Constants.VELOCITY_PID_UPDATE_RATE, Constants.STEERING_PID_UPDATE_RATE)

  #-------------------------------------------------------------------------------
  def drive(self):
    '''
    Autonomously drives the vehicle based on the control goals
    '''
    self.currentTime = time.time()
    self.updateMeasurements()
    self.velocityDuration = 1.0 - (max(min(self.velocityPID.control(), 0.5), -0.5) + 0.5)
    self.steeringDuration = max(min(self.steeringAnglePID.control(), 0.5), -0.5) + 0.5
    # TODO: Do we need bound checks on velocity and steering angles?
    self.controlChnl(Constants.PWM_DRIVE_CHNL, self.velocityDuration)
    self.controlChnl(Constants.PWM_TURN_CHNL, self.steeringDuration)

    sleepTime = (1.0 / self.updateRate) - (time.time() - self.currentTime)
    if sleepTime > self.updateRate:
      time.sleep(sleepTime)

    self.totalUpdateTime += self.updateRate

  #-------------------------------------------------------------------------------
  def updateGoals(self, velocityGoal, steeringAngleGoal):
    '''
    Updates the vehicle's state
    @param velocityGoal - the velocity we are trying to achieve
    @param steeringAngleGoal - the steering angle we are trying to achieve
    '''
    # Set PID Goals
    self.velocityPID.setGoal(velocityGoal)
    self.steeringAnglePID.setGoal(steeringAngleGoal)

  #-------------------------------------------------------------------------------
  def updateMeasurements(self):
    '''
    Updates PID measurements
    '''
    self.updateVelocityMeasurement()
    self.updateSteeringMeasurement()

  #-------------------------------------------------------------------------------
  def updateVelocityMeasurement(self):
    '''
    Update the velocity PID measurement
    '''
    if self.totalUpdateTime >= Constants.VELOCITY_PID_UPDATE_RATE:
      # TODO: Test. Threading issues
      self.totalStripCount = self.sensorConversionThread.totalStripCount
      dt = self.currentTime - self.prevTime
      if dt == 0.0:
        dt = 0.00000001
      vehicleVelocity = (self.totalStripCount - self.prevTotalStripCount) / dt * Constants.STRIP_COUNT_TO_METERS
      self.velocityPID.setMeasurement(vehicleVelocity)
      self.prevTime = self.currentTime

  #-------------------------------------------------------------------------------
  def updateSteeringMeasurement(self):
    '''
    Update the steering PID measurement
    '''
    if self.totalUpdateTime >= Constants.STEERING_PID_UPDATE_RATE:
      # TODO: Test. Threading issues
      vehicleSteeringAngle = self.sensorConversionThread.steeringAngle
      self.velocityPID.setMeasurement(vehicleSteeringAngle)

  #-------------------------------------------------------------------------------
  def controlChnl(self, chnl, pulse):
    '''
    Controls the PWM channel
    @param chnl - PWM channel to send signal to
    @param pulse - the pulse length to send
    '''
    if pulse > Constants.PWM_MAX_PULSE:
      pulse = Constants.PWM_MAX_PULSE
    if pulse < Constants.PWM_MIN_PULSE:
      pulse = Constants.PWM_MIN_PULSE
    # TODO: What is this frequency equation doing?
    self.pwm.set_pwm(chnl, 0, int(pulse / Constants.PWM_SENSOR_FREQ * 10000000.0 * 5.0 / 6.0))

  #-------------------------------------------------------------------------------
  def ramp(self, chnl, curVal, rampVal, rampTime, inc):
    '''
    Ramps the channel gradually to the specified value
    @param chnl - PWM channel to control
    @param curVal - the current value of the pwm channel
    @param rampVal - the value to ramp to
    @param rampTime - the time to ramp to the given value
    @param inc - the increment to ramp by
    '''
    pulse = (rampVal - curVal) / inc
    for i in range(0, inc):
      curVal += pulse
      self.controlChnl(chnl, curVal)
      time.sleep(rampTime / inc)

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
    desc += "{0}\tvelocityDuration = {1}\n".format('\t' * self.tabs, self.velocityDuration)
    desc += "{0}\tsteeringDuration = {1}\n".format('\t' * self.tabs, self.steeringDuration)
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

