from threading import Thread

class DataConsumerThread(Thread):
  '''
  Overrides the Thread class to consume data
  '''
  #-------------------------------------------------------------------------------
  def __init__(self, group=None, target=None, name=None, args=(), kwargs={}, daemon=None, serial=None):
    '''
    Initializes the data consumer thread
    @param Refer to the Python Thread class for documentation on all thread specific parameters
    @param serial - the serial communication line
    '''
    super(DataConsumerThread, self).__init__(group=group, target=target, name=name, daemon=daemon)
    self.args = args
    self.kwargs = kwargs
    self.serial = serial
    self.shutDown = False

  #-------------------------------------------------------------------------------
  def run(self):
    '''
    Consumer function. This is the target of the thread that will be run continously until shutdown. This consumes data off of the serial line between the Arduino and the Raspberry Pi as well as any sensors connected to the Raspberry Pi
    @param serial - the serial communication line
    '''
    print("DBG: consumer function called")
    while not self.shutDown:
      print("DBG: consumer attempting to read serial line")
      line = self.serial.read(4)
      print("Received: '{0}'".format(line))

  #-------------------------------------------------------------------------------
  def shutdown(self):
    '''
    Shutdown the consumer thread
    '''
    print("DBG: Shuttingdown consumer thread")
    self.shutDown = True

