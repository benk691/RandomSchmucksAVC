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

  #-------------------------------------------------------------------------------
  def run(self):
    '''
    Consumer function. This is the target of the thread that will be run continously until shutdown. 
    This consumes data off of the I2C bus as well as the distance sensors
    '''
    while not self.shutDown:
     self.sensors.read() 

  #-------------------------------------------------------------------------------
  def shutdown(self):
    '''
    Shutdown the consumer thread
    '''
    self.shutDown = True

  #-------------------------------------------------------------------------------
  def __del__(self):
    '''
    Destructor
    '''
    self.join(timeout=5)

