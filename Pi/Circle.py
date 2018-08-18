
class Circle:
  '''
  General circle definition
  '''
  #-------------------------------------------------------------------------------
  def __init__(self, center, radius, startAngle, stopAngle):
    '''
    Initializes the circle
    @param center - the center (X, Y) of the circle
    @radius - the circle radius
    @startAngle - the start angle to start drawing the circle [radians]
    @stopAngle - the stop angle to stop drawing the circle [radians]
    '''
    self.center = center
    self.radius = radius
    self.startAngle = startAngle
    self.stopAngle = stopAngle
    self.tabs = 0

  #-------------------------------------------------------------------------------
  def intersect(self, line):
    '''
    Determines if the circle intersects with the given line
    @param line - the line to determine intersection with
    @return true if there is intersection, otherwise returns false
    '''
    pass

  #-------------------------------------------------------------------------------
  def distance(self, obj):
    '''
    Determine distance to the given object from the circle
    @param obj - Line, point (x,y), Circle
    @return distance to object
    '''
    pass

  #-------------------------------------------------------------------------------
  def setTabs(self, tabs):
    '''
    Sets the number of tabs used for print out
    '''
    self.tabs = tabs

  #-------------------------------------------------------------------------------
  def _debugDescription(self):
    '''
    Generates debugging information about the circle
    @return string describing debug information
    '''
    desc = "{0}Circle:\n".format('\t' * self.tabs)
    desc += "{0}\tCenter = {1}\n".format('\t' * self.tabs, self.center)
    desc += "{0}\tRadius = {1}\n".format('\t' * self.tabs, self.radius)
    desc += "{0}\tangleRange(start, stop) = ({1}, {2})\n".format('\t' * self.tabs, self.startAngle, self.stopAngle)
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

