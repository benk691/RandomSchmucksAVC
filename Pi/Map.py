from Circle import Circle
from Line import Line

class Map:
  '''
  Base map class
  '''
  def __init__(self):
    '''
    Initializes the course map
    '''
    # Course Circles
    self.circles = []
    # Course Lines
    self.lines = []
    # Course Waypoints
    self.waypoints = []

  #-------------------------------------------------------------------------------
  def _debugDescription(self):
    '''
    Generates debugging information about the course map
    @return string describing debug information
    '''
    tabs = 1
    for c in self.circles:
      c.setTabs(tabs)

    for l in self.lines:
      l.setTabs(tabs)
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

