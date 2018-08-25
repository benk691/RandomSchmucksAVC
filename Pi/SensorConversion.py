import Constants
import math
import time
from Filter import Filter
from threading import Thread

class SensorConversion(Thread):
  '''
  Converts the incoming raw sensor values.
  Tachometer => Convert to a velocity
  Potentiometer => Steering Angle
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
    self._prevRightTachValue = 0
    self._prevLeftTachValue = 0
    self._leftTachValue = -0.0
    self._rightTachValue = -0.0
    self._distStartTime = 0
    self._rightHigh = 0
    self._leftHigh = 0
    self._rightStripCount = 0.0
    self._leftStripCount = 0.0
    self._elapsedTime = 0
    self._startTime = 0
    self._rightVelocity = 0.0
    self._leftVelocity = 0.0

  #-------------------------------------------------------------------------------
  def run(self):
    '''
    Runs the conversion thread
    '''
    self._distStartTime = time.time()
    self._startTime = time.time()
    while not self.shutDown:
      self.getSensorValues()

      # TODO: Modularize this
      if (self._loopCount % 1 == 0):
        self._prevRightTachValue = self.rightTachValue;
        self._prevLeftTachValue = self.leftTachValue;

      self.distTraveled += (self.velocity / (time.time() - self._distStartTime))

      # TODO: Clean this up
      self.steeringPotValue = self.steeringFilter.filter(self.steeringPotValue)

      if self._rightHigh == 0 and self._rightTachValue > Constants.TACH_RIGHT_THRESHOLD_HIGH:
        self._rightStripCount += 0.5
        self._rightHigh = 1

      # TODO: Should the comparison be with low instead of high?
      if self._rightHigh == 1 and self._rightTachValue < Constants.TACH_RIGHT_THRESHOLD_HIGH:
        self._rightStripCount += 0.5
        self._rightHigh = 0

      if self._leftHigh == 0 and self._leftTachValue > Constants.TACH_LEFT_THRESHOLD_HIGH:
        self._leftStripCount += 0.5
        self._leftHigh = 1

      # TODO: Should the comparison be with low instead of high?
      if self._leftHigh == 1 and self._leftTachValue < Constants.TACH_LEFT_THRESHOLD_HIGH:
        self._leftStripCount += 0.5
        self._leftHigh = 0

      self._loopCount += 1

      if self._loopCount >= Constants.MAX_LOOP_COUNT:
        self._elapsedTime = (time.time() * 1000) - self._startTime
        self._startTime = (time.time() * 1000)
        self._rightVelocity = ((self._rightStripCount / 30.0) * 0.36 * math.pi) / (self._elapsedTime / 1000.0)
        self._leftVelocity = ((self._leftStripCount / 30.0) * 0.36 * math.pi) / (self._elapsedTime / 1000.0)

        self.velocity = (self._leftVelocity + self._rightVelocity) / 2.0
        self.velocity = self.velocityFilter.filter(self.velocity)


  #-------------------------------------------------------------------------------
  def getSensorValues(self):
    '''
    Get the raw sensor values
    '''
    self.steeringPotValue = self.dataConsumerThread.sensors.steeringPotValue
    self.leftTachValue =  self.dataConsumerThread.sensors.leftTachValue
    self.rightTachValue = self.dataConsumerThread.sensors.rightTachValue
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
  def shutdown(self):
    '''
    Shutdown the thread
    '''
    self.shutDown = True

  #-------------------------------------------------------------------------------
  def __del__(self):
    '''
    Destructor
    '''
    self.join(timeout=5)

