import math
import Constants

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
  def findIntersection(self, line):
    '''
    Determines if and where the circle intersects with the given line
    Code Credit: Stack Overflow circle line segment collision detection algorithm by Duq
    @param line - the line to determine intersection with
    @return the point where the line intersects with the circle, or None if line does not intersect
    '''
    # shift line points
    shiftedStartPoint = [ line.startPoint[Constants.X] - self.center[Constants.X], line.startPoint[Constants.Y] - self.center[Constants.Y] ]
    shiftedEndPoint = [ line.endPoint[Constants.X] - self.center[Constants.X], line.endPoint[Constants.Y] - self.center[Constants.Y] ]

    # Slope of the line
    m = (shiftedEndPoint[Constants.Y] - shiftedStartPoint[Constants.Y]) / (shiftedEndPoint[Constants.X] - shiftedStartPoint[Constants.X])
    # Y-Intercept of line
    b = shiftedStartPoint[Constants.Y] - m * shiftedStartPoint[Constants.X]
    # Value to be square rooted
    underRadical = math.pow(self.radius, 2) * math.pow(m, 2) + math.pow(self.radius, 2) - math.pow(b ,2)
    if underRadical < 0.0:
      # Line completely missed
      return None
    else:
      # one of the intercept x's
      t1 = (-m * b + math.sqrt(underRadical))/(math.pow(m, 2) + 1)
      # other intercept's
      t2 = (-m * b - math.sqrt(underRadical))/(math.pow(m, 2) + 1)
      # intercept point 1
      i1 = [ t1 + self.center[Constants.X], m * t1 + b + self.center[Constants.Y] ]
      # intercept point 2
      i2 = [ t2 + self.center[Constants.X], m * t2 + b + self.center[Constants.Y] ]
      # TODO: angle accounting and closest point considerations
      return [i1, i2]

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

