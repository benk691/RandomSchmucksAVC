import math
import time
import Constants
import math
import time
from GeneralFunctions import *
from Filter import Filter
from threading import Thread

class SensorConversion(Thread):
  '''
  Converts the incoming raw sensor values.
  Tachometer => Convert to a velocity (m/s)
  Potentiometer => Steering Angle (radians)
  Distance => No change needed
  IMU => No change needed
  After converting the values the filter of the values is applied
  '''
  #-------------------------------------------------------------------------------
  def __init__(self, group=None, target=None, name=None, args=(), kwargs={}, daemon=None, dataConsumerThread=None):
    '''
    Initializes the sensor conversion
    @param Refer to the Python Thread class for documentation on all thread specific parameters
    '''
    super(SensorConversion, self).__init__(group=group, target=target, name=name, daemon=daemon)
    self.args = args
    self.kwargs = kwargs
    self.dataConsumerThread = dataConsumerThread
    self.shutDown = False
    self.steeringPotValue = -0.0
    self.velocity = -0.0
    # NOTE: Angle is in radians
    self.steeringAngle = -0.0
    self.leftDistance = -0.0
    self.rightDistance = -0.0
    self.heading = -0.0
    self.roll = -0.0
    self.pitch = -0.0
    self.sysCal = -0.0
    self.gyroCal = -0.0
    self.accelCal = -0.0
    self.magCal = -0.0
    self.distTraveled = 0.0
    self.velocityFilter = Filter(Constants.VELOCITY_FILTER_A)
    self.steeringFilter = Filter(Constants.STEERING_FILTER_A)
    self._loopCount = 0
    self._prevRightTachValue = -0.0
    self._prevLeftTachValue = -0.0
    self._leftTachValue = -0.0
    self._rightTachValue = -0.0
    self._distStartTime = 0
    self._rightHigh = 0
    self._leftHigh = 0
    self._rightStripCount = -0.0
    self._leftStripCount = -0.0
    self._elapsedTime = 0
    self._startTime = 0
    self._rightVelocity = -0.0
    self._leftVelocity = -0.0

  #-------------------------------------------------------------------------------
  def run(self):
    '''
    Runs the conversion thread
    '''
    self._startTime = time.time()
    while not self.shutDown:
      self._getSensorValues()

      # TODO: Modularize this
      # TODO: Clean this up
      # TODO: Convert the pot value into a steering anlog that is in radians
      self.steeringPotValue = self.steeringFilter.filter(self.steeringPotValue)

      # TODO: Move this into the consumer thread
      self._calculateStripCount()
      
      self._loopCount += 1

      if self._loopCount >= Constants.MAX_LOOP_COUNT:
        self._calculateVelocity()

        # Reset counts
        self._loopCount = 0
        self._rightStripCount = 0.0
        self._leftStripCount = 0.0

  #-------------------------------------------------------------------------------
  def shutdown(self):
    '''
    Shutdown the thread
    '''
    self.shutDown = True

  #-------------------------------------------------------------------------------
  def _calculateVelocity(self):
    '''
    Calculates the vehicles velocity in m/s
    '''
    self._elapsedTime = convertSecToMilliSec(time.time()) - self._startTime
    self._startTime = convertSecToMilliSec(time.time())
    # TODO: What is 0.36? wheel diameter in meters
    self._rightVelocity = ((self._rightStripCount / Constants.TACH_TOTAL_STRIPS) * 0.36 * math.pi) / convertMilliSecToSec(self._elapsedTime)
    self._leftVelocity = ((self._leftStripCount / Constants.TACH_TOTAL_STRIPS) * 0.36 * math.pi) / convertMilliSecToSec(self._elapsedTime)

    # Take the average of the left and right velocity to get the vehicle velocity
    self.velocity = (self._leftVelocity + self._rightVelocity) / 2.0
    self.velocity = self.velocityFilter.filter(self.velocity)

    self.distTraveled = ((self._totalLeftStripCount + self._totalRightStripCount) / 2.0) / Constants.TACH_TOTAL_STRIPS) * 0.36 * math.pi)

  #-------------------------------------------------------------------------------
  def _calculateStripCount(self):
    '''
    Calculates the new strip counts on the left and right side of the vehicle
    '''
    if self._rightHigh == 0 and self._rightTachValue > Constants.TACH_RIGHT_THRESHOLD_HIGH:
      self._rightStripCount += 0.5
      self._totalRightStripCount += self._rightStripCount
      self._rightHigh = 1

    # TODO: Changed the comparison to low, needs testing
    if self._rightHigh == 1 and self._rightTachValue < Constants.TACH_RIGHT_THRESHOLD_LOW:
      self._rightStripCount += 0.5
      self._totalRightStripCount += self._rightStripCount
      self._rightHigh = 0

    if self._leftHigh == 0 and self._leftTachValue > Constants.TACH_LEFT_THRESHOLD_HIGH:
      self._leftStripCount += 0.5
      self._totalLeftStripCount += self._rightStripCount
      self._leftHigh = 1

    # TODO: Changed the comparison to low, needs testing
    if self._leftHigh == 1 and self._leftTachValue < Constants.TACH_LEFT_THRESHOLD_LOW:
      self._leftStripCount += 0.5
      self._totalLeftStripCount += self._rightStripCount
      self._leftHigh = 0

  #-------------------------------------------------------------------------------
  def _getSensorValues(self):
    '''
    Get the raw sensor values
    '''
    self.steeringPotValue = self.dataConsumerThread.sensors.steeringPotValue
    self._leftTachValue =  self.dataConsumerThread.sensors.leftTachValue
    self._rightTachValue = self.dataConsumerThread.sensors.rightTachValue
    self.leftDistance = self.dataConsumerThread.sensors.leftDistance
    self.rightDistance = self.dataConsumerThread.sensors.rightDistance
    self.heading = self.dataConsumerThread.sensors.heading
    self.roll = self.dataConsumerThread.sensors.roll
    self.pitch = self.dataConsumerThread.sensors.pitch
    self.sysCal = self.dataConsumerThread.sensors.sysCal
    self.gyroCal = self.dataConsumerThread.sensors.gyroCal
    self.accelCal = self.dataConsumerThread.sensors.accelCal
    self.magCal = self.dataConsumerThread.sensors.magCal

  #-------------------------------------------------------------------------------
  def _debugDescription(self):
    '''
    Generates debugging information about the sensor conversion
    @return string describing debug information
    '''
    self.velocityFilter.setTabs(1)
    self.steeringFilter.setTabs(1)
    desc = "Sensor Conversion:\n"
    desc += "\tshutDown = {0}\n".format(self.shutDown)
    desc += "\tsteeringPotValue = {0}\n".format(self.steeringPotValue)
    desc += "\tvelocity = {0}\n".format(self.velocity)
    desc += "\tsteeringAngle = {0}\n".format(math.degrees(self.steeringAngle))
    desc += "\tleftDistance = {0}\n".format(self.leftDistance)
    desc += "\trightDistance = {0}\n".format(self.rightDistance)
    desc += "\theading = {0}\n".format(self.heading)
    desc += "\troll = {0}\n".format(self.roll)
    desc += "\tpitch = {0}\n".format(self.pitch)
    desc += "\tsysCal = {0}\n".format(self.sysCal)
    desc += "\tgyroCal = {0}\n".format(self.gyroCal)
    desc += "\taccelCal = {0}\n".format(self.accelCal)
    desc += "\tmagCal = {0}\n".format(self.magCal)
    desc += "\tdistTraveled = {0}\n".format(self.distTraveled)
    desc += "\tvelocityFilter = {0}\n".format(self.velocityFilter)
    desc += "\tsteeringFilter = {0}\n".format(self.steeringFilter)
    desc += "\t_loopCount = {0}\n".format(self._loopCount)
    desc += "\t_leftTachValue = {0}\n".format(self._leftTachValue)
    desc += "\t_rightTachValue = {0}\n".format(self._rightTachValue)
    desc += "\t_rightHigh = {0}\n".format(self._rightHigh)
    desc += "\t_leftHigh = {0}\n".format(self._leftHigh)
    desc += "\t_rightStripCount = {0}\n".format(self._rightStripCount)
    desc += "\t_leftStripCount = {0}\n".format(self._leftStripCount)
    desc += "\t_elapsedTime = {0}\n".format(self._elapsedTime)
    desc += "\t_startTime = {0}\n".format(self._startTime)
    desc += "\t_rightVelocity = {0}\n".format(self._rightVelocity)
    desc += "\t_leftVelocity = {0}\n".format(self._leftVelocity)
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

  #-------------------------------------------------------------------------------
  def __del__(self):
    '''
    Destructor
    '''
    self.join(timeout=5)

