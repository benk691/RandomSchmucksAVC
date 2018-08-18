
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
    self.tabs = 0

  #-------------------------------------------------------------------------------
  def setTabs(self, tabs):
    '''
    Sets the number of tabs used for print out
    '''
    self.tabs = tabs

  #-------------------------------------------------------------------------------
  def _debugDescription(self):
    '''
    Generates debugging information about the line
    @return string describing debug information
    '''
    desc = "{0}Line:\n".format('\t' * self.tabs)
    desc += "{0}\tstartPoint = {1}\n".format('\t' * self.tabs, self.startPoint)
    desc += "{0}\tendPoint = {1}\n".format('\t' * self.tabs, self.endPoint)
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

