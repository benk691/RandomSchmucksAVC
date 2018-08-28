import statistics

class MedianFilter:
  '''
  Filters measurements through a median filter
  '''
  #-------------------------------------------------------------------------------
  def __init__(self, order):
    '''
    Initializes the median filter
    @param order - the order of the median filter, or the number of measurements to take the median of
    '''
    self.order = order
    self.measurementQueue = []
    self.measurement = -0.0
    self.medianValue = -0.0

  #-------------------------------------------------------------------------------
  def filter(self, measurement):
    '''
    Filters the received measurement
    @param measurement - the measurement to filter
    @return median of the queue with the measurement pass in making sure the queue 
    does not exceed the specified order
    '''
    self.measurement = measurement
    if self.order <= 1:
      self.medianValue = self.measurement
      return self.medianValue
    if len(self.measurementQueue) >= self.order:
      # Remove the oldest measurements from the queue
      while len(self.measurementQueue) >= self.order:
        self.measurementQueue.pop(0)
    self.measurementQueue.append(self.measurement)
    self.medianValue = statistics.median(map(float, self.measurementQueue))
    return self.medianValue

  #-------------------------------------------------------------------------------
  def _debugDescription(self):
    '''
    Generates debugging information about the filter
    @return string describing debug information
    '''
    desc = "MedianFilter:\n"
    desc += "\torder = {0}\n".format(self.order)
    desc += "\tmeasurement = {0}\n".format(self.measurement)
    desc += "\tmedianValue = {0}\n".format(self.medianValue)
    desc += "\tmeasurementQueue = {0}\n".format(self.measurementQueue)
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

