from Constants import Constants

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
  def findIntersection(self, line):
    '''
    Determines if and where the line intersects with the given line
    Code Credit: Stack Overflow how do you detect where two line segments intersect by Kris
    @param line - the line to determine intersection with
    @return the point where the line intersects with the line, or None if line does not intersect
    '''
    myXDiff = self.endPoint[Constants.X] - self.startPoint[Constants.X]
    myYDiff = self.endPoint[Constants.Y] - self.startPoint[Constants.Y]
    inXDiff = line.endPoint[Constants.X] - line.startPoint[Constants.X]
    inYDiff = line.endPoint[Constants.Y] - line.startPoint[Constants.Y]

    denom = myXDiff * inYDiff - inXDiff * myYDiff

    if denom == 0: 
      # collinear
      return None 
    
    denomIsPositive = denom > 0

    xDiff = self.startPoint[Constants.X] - line.startPoint[Constants.X]
    yDiff = self.startPoint[Constants.Y] - line.startPoint[Constants.Y]

    sNumer = myXDiff * yDiff - myYDiff * xDiff

    if (sNumer < 0) == denomIsPositive:
      # no collision
      return None

    tNumer = inXDiff * yDiff - inYDiff * xDiff

    if (tNumer < 0) == denomIsPositive:
      # no collision
      return None

    if (sNumer > denom) == denomIsPositive or (tNumer > denom) == denomIsPositive:
      # no collision
      return None

    # collsion detected
    t = tNumer / denom
    intersectionPoint = [ self.startPoint[Constants.X] + (t * myXDiff), self.startPoint[Constants.Y] + (t * myYDiff) ]
    return intersectionPoint

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

