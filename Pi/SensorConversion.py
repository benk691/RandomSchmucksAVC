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
    self.leftTachValue = -0.0
    self.rightTachValue = -0.0
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
    # TODO: Do we want a different filter value for all sensors?
    self.filter = Filter(0.0)

  #-------------------------------------------------------------------------------
  def run(self):
    '''
    Runs the conversion thread
    '''
    while not self.shutDown:
      self.getSensorValues()     

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

