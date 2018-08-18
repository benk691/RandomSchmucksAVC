
class Line:
  '''
  General line definition
  '''
  #-------------------------------------------------------------------------------
  def __init__(self, startPoint, endPoint):
    '''
    Initializes the line
    @param startPoint - the line start point (x, y)
    @param endPoint - the line end point (x, y)
    '''
    self.startPoint = startPoint
    self.endPoint = endPoint

  #-------------------------------------------------------------------------------
  def _debugDescription(self):
    '''
    Generates debugging information about the line
    @return string describing debug information
    '''
    desc = "Line:\n"
    desc += "\tstartPoint = {0}\n".format(self.startPoint)
    desc += "\tendPoint = {0}\n".format(self.endPoint)
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

