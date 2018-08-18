
class Circle:
  '''
  General circle definition
  '''
  #-------------------------------------------------------------------------------
  def __init__(self, centerX, centerY, radius, startAngle, stopAngle):
    '''
    Initializes the circle
    @param centerX - the X coordinate of the center of the circle
    @param centerY - the Y coordinate of the center of the circle
    @radius - the circle radius
    @startAngle - the start angle to start drawing the circle
    @stopAngle - the stop angle to stop drawing the circle 
    '''
    self.centerX = centerX
    self.centerY = centerY
    self.radius = radius
    self.startAngle = startAngle
    self.stopAngle = stopAngle

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
  def _debugDescription(self):
    '''
    Generates debugging information about the circle
    @return string describing debug information
    '''
    desc = "Circle:\n"
    desc += "\tCenter(X, Y) = ({0}, {1})\n".format(self.centerX, self.centerY)
    desc += "\tRadius = {0}\n".format(self.radius)
    desc += "\tangleRange(start, stop) = ({0}, {1})\n".format(self.startAngle, self.stopAngle)
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
