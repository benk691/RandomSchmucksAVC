import math
import time
import Constants
from GeneralFunctions import GeneralFunctions
from IIRFilter import IIRFilter
from MedianFilter import MedianFilter
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
    # TODO: Do we need individual constants for each sensor or just one per filter?
    self.velocityMedianFilter = MedianFilter(Constants.VELOCITY_MEDIAN_FILTER_ORDER)
    self.steeringMedianFilter = MedianFilter(Constants.STEERING_MEDIAN_FILTER_ORDER)
    self.leftDistMedianFilter = MedianFilter(Constants.DIST_MEDIAN_FILTER_ORDER)
    self.rightDistMedianFilter = MedianFilter(Constants.DIST_MEDIAN_FILTER_ORDER)
    self.velocityIirFilter = IIRFilter(Constants.VELOCITY_IIR_FILTER_A)
    self.steeringIirFilter = IIRFilter(Constants.STEERING_IIR_FILTER_A)
    self.distIirFilter = IIRFilter(Constants.DIST_IIR_FILTER_A)
    # TODO: Distance Filter IIR
    # Check dist >= MAX then set to # higher than max then feed it into filter
    self._loopCount = 0
    self._rightStripCount = -0.0
    self._leftStripCount = -0.0
    self._totalRightStripCount = -0.0
    self._totalLeftStripCount = -0.0
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

      self._filterValues()

      # TODO: Convert the pot value into a steering anlog that is in radians

      self._loopCount += 1

      if self._loopCount >= Constants.MAX_LOOP_COUNT:
        self._calculateVelocity()

        # Reset counts
        self._loopCount = 0
        self._rightStripCount = 0.0
        self._leftStripCount = 0.0
        # TODO: I forsee threading complications with this. Needs testing
        self.dataConsumerThread.rightStripCount = 0.0
        self.dataConsumerThread.leftStripCount = 0.0

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
    self._rightVelocity = ((self._rightStripCount / Constants.TACH_TOTAL_STRIPS) * Constants.VEHICLE_WHEEL_DIAMETER * math.pi) / convertMilliSecToSec(self._elapsedTime)
    self._leftVelocity = ((self._leftStripCount / Constants.TACH_TOTAL_STRIPS) * Constants.VEHICLE_WHEEL_DIAMETER * math.pi) / convertMilliSecToSec(self._elapsedTime)

    # Take the average of the left and right velocity to get the vehicle velocity
    self.velocity = (self._leftVelocity + self._rightVelocity) / 2.0
    self.velocity = self.velocityMedianFilter.filter(self.velocity)
    self.velocity = self.velocityIirFilter.filter(self.velocity)

    self.distTraveled = ((self._totalLeftStripCount + self._totalRightStripCount) / 2.0) / Constants.TACH_TOTAL_STRIPS) * Constants.VEHICLE_WHEEL_DIAMETER * math.pi)

  #-------------------------------------------------------------------------------
  def _getSensorValues(self):
    '''
    Get the raw sensor values
    '''
    self.steeringPotValue = self.dataConsumerThread.sensors.steeringPotValue
    # TODO: I forsee threading complication with this. Needs testing
    self._leftStripCount =  self.dataConsumerThread.leftStripCount
    self._rightStripCount = self.dataConsumerThread.rightStripCount
    self._totalLeftStripCount =  self.dataConsumerThread.totalLeftStripCount
    self._totalRightStripCount = self.dataConsumerThread.totalRightStripCount
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
  def _filterValues(self):
    '''
    Filter values that are read in
    '''
    # Filter Steering Pot Value
    self.steeringPotValue = self.steeringMedianFilter.filter(self.steeringPotValue)
    self.steeringPotValue = self.steeringIirFilter.filter(self.steeringPotValue)
    # Filter Left Distance Value
    self.leftDistance = self.leftDistMedianFilter.filter(self.leftDistance)
    self.leftDistance = self.distIirFilter.filter(self.leftDistance)
    # Filter Right Distance Value
    self.rightDistance = self.rightDistMedianFilter.filter(self.rightDistance)
    self.rightDistance = self.distIirFilter.filter(self.rightDistance)

  #-------------------------------------------------------------------------------
  def _debugDescription(self):
    '''
    Generates debugging information about the sensor conversion
    @return string describing debug information
    '''
    # TODO: Move all setTabs calls to contructor, only needs to be done once
    self.velocityIirFilter.setTabs(1)
    self.steeringIirFilter.setTabs(1)
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
    desc += "\tvelocityIirFilter = {0}\n".format(self.velocityIirFilter)
    desc += "\tsteeringIirFilter = {0}\n".format(self.steeringIirFilter)
    desc += "\t_loopCount = {0}\n".format(self._loopCount)
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

