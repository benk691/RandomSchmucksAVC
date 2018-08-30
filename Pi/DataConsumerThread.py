import Constants
from Sensors import Sensors
from threading import Thread

class DataConsumerThread(Thread):
  '''
  Overrides the Thread class to consume data
  '''
  #-------------------------------------------------------------------------------
  def __init__(self, group=None, target=None, name=None, args=(), kwargs={}, daemon=None):
    '''
    Initializes the data consumer thread
    @param Refer to the Python Thread class for documentation on all thread specific parameters
    '''
    super(DataConsumerThread, self).__init__(group=group, target=target, name=name, daemon=daemon)
    self.args = args
    self.kwargs = kwargs
    self.shutDown = False
    self.sensors = Sensors()
    self.rightStripCount = -0.0
    self.leftStripCount = -0.0
    self.totalRightStripCount = -0.0
    self.totalLeftStripCount = -0.0
    self._rightHigh = 0
    self._leftHigh = 0

  #-------------------------------------------------------------------------------
  def run(self):
    '''
    Consumer function. This is the target of the thread that will be run continously until shutdown. 
    This consumes data off of the I2C bus as well as the distance sensors
    '''
    while not self.shutDown:
     self.sensors.read() 
     self._calculateStripCount()

  #-------------------------------------------------------------------------------
  def shutdown(self):
    '''
    Shutdown the consumer thread
    '''
    self.shutDown = True

  #-------------------------------------------------------------------------------
  def _calculateStripCount(self):
    '''
    Calculates the new strip counts on the left and right side of the vehicle
    '''
    if self._rightHigh == 0 and self.sensors.rightTachValue > Constants.TACH_RIGHT_THRESHOLD_HIGH:
      self.rightStripCount += 0.5
      self.totalRightStripCount += self.rightStripCount
      self._rightHigh = 1

    # TODO: Changed the comparison to low, needs testing
    if self._rightHigh == 1 and self.sensors.rightTachValue < Constants.TACH_RIGHT_THRESHOLD_LOW:
      self.rightStripCount += 0.5
      self.totalRightStripCount += self.rightStripCount
      self._rightHigh = 0

    if self._leftHigh == 0 and self.sensors.leftTachValue > Constants.TACH_LEFT_THRESHOLD_HIGH:
      self.leftStripCount += 0.5
      self.totalLeftStripCount += self.rightStripCount
      self._leftHigh = 1

    # TODO: Changed the comparison to low, needs testing
    if self._leftHigh == 1 and self.sensors.leftTachValue < Constants.TACH_LEFT_THRESHOLD_LOW:
      self.leftStripCount += 0.5
      self.totalLeftStripCount += self.rightStripCount
      self._leftHigh = 0

  #-------------------------------------------------------------------------------
  def _debugDescription(self):
    '''
    Generates debugging information about the data consumer thread
    @return string describing debug information
    '''
    desc = "DataConsumerThread:\n"
    desc += "\tshutDown = {0}\n".format(self.shutDown)
    # TODO: Add setTabs function in Sensors
    desc += "\tsensors = {0}\n".format(self.sensors)
    desc += "\trightStripCount = {0}\n".format(self.rightStripCount)
    desc += "\tleftStripCount = {0}\n".format(self.leftStripCount)
    desc += "\ttotalRightStripCount = {0}\n".format(self.totalRightStripCount)
    desc += "\ttotalLeftStripCount = {0}\n".format(self.totalLeftStripCount)
    desc += "\t_rightHigh = {0}\n".format(self._rightHigh)
    desc += "\t_leftHigh = {0}\n".format(self._leftHigh)
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

