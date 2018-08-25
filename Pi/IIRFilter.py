
class IIRFilter:
  '''
  Filters measurements to reduce smooth out noisy data. This is essentially a low pass filter
  '''
  #-------------------------------------------------------------------------------
  def __init__(self, A):
    '''
    Initializes the IIR filter
    @param A - coefficient between [0,1] (degree to which it listens to input [throws error if not between 0 and 1])
    '''
    if A < 0.0 or A > 1.0:
      raise ValueError("Invalid value of A for Filter class. A must be between [0,1].")
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
    desc = "IIRFilter:\n"
    desc += "\tA = {0}\n".format(self.A)
    desc += "\tprevY = {0}\n".format(self.prevY)
    desc += "\tY = {0}\n".format(self.Y)
    desc += "\tmeasurement = {0}\n".format(self.measurement)
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

