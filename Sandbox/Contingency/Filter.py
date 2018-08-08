
class Filter:
  #-------------------------------------------------------------------------------
  def __init__(self, a):
    # all sensors feed into filter
    self.y = 0.0
    self.a = a
    self.prevY = "DEAD"
    self.inputVal = 0.0

  #-------------------------------------------------------------------------------
  def recvMeasurement(self, measurement):
    if self.prevY == "DEAD":
      self.prevY = measurement
    self.inputVal = measurement

  #-------------------------------------------------------------------------------
  def filter(self):
    # Y - output signal (filtered value)
    # A - coefficient between 0- 1 (degree to which it listens to input(throw error if not between 0 and 1)
    self.y = (1 - self.a) * self.inputVal + self.a * self.prevY
    self.prevY = self.y
    # output Y or return Y
    return self.y

  #-------------------------------------------------------------------------------
  def _debugDescription(self):
    #desc = "Fil
    pass
