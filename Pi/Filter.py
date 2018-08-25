
class Filter:
  '''
  Filters measurements to reduce noisy data
  '''
  #-------------------------------------------------------------------------------
  def __init__(self, A):
    '''
    Initializes the filter
    @param a - coefficient between [0,1] (degree to which it listens to input [throws error if not between 0 and 1])
    '''
    if A < 0.0 or A > 1.0:
      raise ValueError("Invalid value of A for Filter class. A must be between [0,1].")
    # all sensors feed into filter
    self.Y = 0.0
    self.A = A
    self.prevY = "DEAD"
    self.measurement = 0.0
    self.tabs = 0

  #-------------------------------------------------------------------------------
  def filter(self, measurement):
    '''
    Filters the received measurement
    @param measurement - the measurement to filter
    @return y - output signal (filtered value)
    '''
    if self.prevY == "DEAD":
      self.prevY = measurement
    self.measurement = measurement
    self.Y = (1 - self.A) * self.measurement + self.A * self.prevY
    self.prevY = self.Y
    return self.Y

  #-------------------------------------------------------------------------------
  def setTabs(self, tabs):
    '''
    Sets the number of tabs used for print out
    @param tabs - number of tabs to use in print out
    '''
    self.tabs = tabs

  #-------------------------------------------------------------------------------
  def _debugDescription(self):
    '''
    Generates debugging information about the filter
    @return string describing debug information
    '''
    desc = "{0}Filter:\n".format('\t' * self.tabs)
    desc += "{0}\tA = {0}\n".format('\t' * self.tabs, self.A)
    desc += "{0}\tprevY = {0}\n".format('\t' * self.tabs, self.prevY)
    desc += "{0}\tY = {0}\n".format('\t' * self.tabs, self.Y)
    desc += "{0}\tmeasurement = {0}\n".format('\t' * self.tabs, self.measurement)
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
