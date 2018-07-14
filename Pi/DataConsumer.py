from DataConsumerThread import DataConsumerThread

class DataConsumer:
  '''
  Consumes data being sent from the Arduino off of the serial line as well as all the data coming from sensors that are directly connected to the Raspberry Pi
  '''
  #-------------------------------------------------------------------------------
  def __init__(self, serial):
    '''
    Initializes the basics of the consumer thread
    @param serial - the serial communication line
    '''
    self.consumerThread = DataConsumerThread(name=self.__class__, daemon=True, serial=serial)

  #-------------------------------------------------------------------------------
  def start(self):
    '''
    Starts the consumer thread
    '''
    print("DBG: Starting consumer thread")
    self.consumerThread.start()

  #-------------------------------------------------------------------------------
  def shutdown(self):
    '''
    Shutdown the consumer thread
    '''
    print("DBG: Shuttingdown consumer thread")
    self.consumerThread.shutdown()

  #-------------------------------------------------------------------------------
  def consumer(self, serial):
    '''
    Consumer function. This is the target of the thread that will be run continously until shutdown. This consumes data off of the serial line between the Arduino and the Raspberry Pi as well as any sensors connected to the Raspberry Pi
    @param serial - the serial communication line
    '''
    print("DBG: consumer function called")
    while not self.shutDown:
      print("DBG: consumer attempting to read serial line")
      line = serial.read(8)
      print("Received: '{0}'".format(line))

  #-------------------------------------------------------------------------------
  def __del__(self):
    '''
    Destructor
    '''
    self.consumerThread.join(timeout=30)

