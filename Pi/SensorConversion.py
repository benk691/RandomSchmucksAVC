import math
import time
import Constants
from multiprocessing import Process
from IIRFilter import IIRFilter
from MedianFilter import MedianFilter
from GeneralFunctions import *

class SensorConversion(Process):
  '''
  Converts the incoming raw sensor values.
  Tachometer => Convert to a velocity (m/s)
  Potentiometer => Steering Angle (radians)
  Distance => No change needed
  IMU => No change needed
  After converting the values the filter of the values is applied
  '''
  #-------------------------------------------------------------------------------
  def __init__(self, group=None, target=None, name=None, args=(), kwargs={}, dataConsumerThread=None):
    '''
    Initializes the sensor conversion
    @param Refer to the Python Thread class for documentation on all thread specific parameters
    '''
    super(SensorConversion, self).__init__(group=group, target=target, name=name)
    self.args = args
    self.kwargs = kwargs
    self.dataConsumerThread = dataConsumerThread
    self.shutDown = False
    self.steeringPotValue = -0.0
    self.totalStripCount = -0.0
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
    self.steeringMedianFilter = MedianFilter(Constants.STEERING_MEDIAN_FILTER_ORDER)
    self.steeringIirFilter = IIRFilter(Constants.STEERING_IIR_FILTER_A)
    self.headingIirFilter = IIRFilter(Constants.HEADING_IIR_FILTER_A)
    self.distIirFilter = IIRFilter(Constants.DIST_IIR_FILTER_A)
    self._prevHeading = 0.0
    self._totalRightStripCount = -0.0
    self._totalLeftStripCount = -0.0

  #-------------------------------------------------------------------------------
  def run(self):
    '''
    Runs the conversion thread
    '''
    while not self.shutDown:
      self._getSensorValues()

      self._filterValues()

      self._convertPotValueToAngle()
      time.sleep(1.0 / 10.0)

  #-------------------------------------------------------------------------------
  def shutdown(self):
    '''
    Shutdown the thread
    '''
    self.shutDown = True

  #-------------------------------------------------------------------------------
  def _getSensorValues(self):
    '''
    Get the raw sensor values
    '''
    self.steeringPotValue = self.dataConsumerThread.sensors.steeringPotValue
    # TODO: Test. I forsee threading complication with this.
    self._totalLeftStripCount =  self.dataConsumerThread.totalLeftStripCount
    self._totalRightStripCount = self.dataConsumerThread.totalRightStripCount

    print("DBG: SC LTSC = {0}".format(self._totalLeftStripCount))
    print("DBG: SC RTSC = {0}".format(self._totalRightStripCount))
    self.totalStripCount = (self._totalLeftStripCount + self._totalRightStripCount) / 2.0

    print("DBG: SC TSC = {0}".format(self.totalStripCount))
    self.leftDistance = self.dataConsumerThread.sensors.leftDistance
    self.rightDistance = self.dataConsumerThread.sensors.rightDistance
    self.heading = Constants.HEADING_WRAP_AROUND - self.dataConsumerThread.sensors.heading
    self.roll = self.dataConsumerThread.sensors.roll
    self.pitch = self.dataConsumerThread.sensors.pitch
    self.sysCal = self.dataConsumerThread.sensors.sysCal
    self.gyroCal = self.dataConsumerThread.sensors.gyroCal
    self.accelCal = self.dataConsumerThread.sensors.accelCal
    self.magCal = self.dataConsumerThread.sensors.magCal

  #-------------------------------------------------------------------------------
  def _filterValues(self):
    '''
    Filter values that are read in
    '''
    # Filter Steering Pot Value
    self.steeringPotValue = self.steeringMedianFilter.filter(self.steeringPotValue)
    self.steeringPotValue = self.steeringIirFilter.filter(self.steeringPotValue)
    # Filter Left Distance Value
    self.leftDistance = self._filterMaxDistance(self.leftDistance)
    self.leftDistance = self.distIirFilter.filter(self.leftDistance)
    # Filter Right Distance Value
    self.rightDistance = self._filterMaxDistance(self.rightDistance)
    self.rightDistance = self.distIirFilter.filter(self.rightDistance)
    # Filter heading
    self.heading, self._prevHeading = unwrapAngle(self.heading, self._prevHeading)
    self.heading = self.headingIirFilter.filter(self.heading)
    # TODO: Need a way to mark values as ready for particle filter
    # Consider the thread safe queue, or a publisher subscriber
    self.heading = math.radians(wrapAngle(self.heading))
    # Velocity is filtered in the _calculateVelocity function

  #-------------------------------------------------------------------------------
  def _filterMaxDistance(self, distance):
    '''
    Perform a sanity check on the distance
    @param distance - the distance read from the sensors
    @return distance that has been sanity checked. If greter than max, then apply a 
            constant value to it so it is more likely to be ignored.
    '''
    if distance >= Constants.DIST_MAX_DISTANCE:
      return Constants.DIST_MAX_DISTANCE + Constants.DIST_MAX_FILTER
    return distance

  #-------------------------------------------------------------------------------
  def _convertPotValueToAngle(self):
    '''
    Convert the steering pot value to a steeing angle [radians]
    '''
    self.steeringAngle = math.radians(Constants.STEERING_CONV_SLOPE * self.steeringPotValue + Constants.STEERING_Y_INTERCEPT)

  #-------------------------------------------------------------------------------
  def _debugDescription(self):
    '''
    Generates debugging information about the sensor conversion
    @return string describing debug information
    '''
    # TODO: Move all setTabs calls to contructor, only needs to be done once
    self.steeringIirFilter.setTabs(1)
    self.distIirFilter.setTabs(1)
    self.headingIirFilter.setTabs(1)
    desc = "Sensor Conversion:\n"
    desc += "\tshutDown = {0}\n".format(self.shutDown)
    desc += "\tsteeringPotValue = {0}\n".format(self.steeringPotValue)
    desc += "\tsteeringAngle = {0}\n".format(math.degrees(self.steeringAngle))
    desc += "\ttotalStripCount = {0}\n".format(self.totalStripCount)
    desc += "\tleftDistance = {0}\n".format(self.leftDistance)
    desc += "\trightDistance = {0}\n".format(self.rightDistance)
    desc += "\theading = {0}\n".format(self.heading)
    desc += "\troll = {0}\n".format(self.roll)
    desc += "\tpitch = {0}\n".format(self.pitch)
    desc += "\tsysCal = {0}\n".format(self.sysCal)
    desc += "\tgyroCal = {0}\n".format(self.gyroCal)
    desc += "\taccelCal = {0}\n".format(self.accelCal)
    desc += "\tmagCal = {0}\n".format(self.magCal)
    desc += "\tsteeringIirFilter = {0}\n".format(self.steeringIirFilter)
    desc += "\tsteeringMedianFilter = {0}\n".format(self.steeringMedianFilter)
    desc += "\tdistIirFilter = {0}\n".format(self.distIirFilter)
    desc += "\theadingIirFilter = {0}\n".format(self.headingIirFilter)
    desc += "\t_prevHeading = {0}\n".format(self._prevHeading)
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
    # TODO: Create timeout constant
    self.join(timeout=5)

