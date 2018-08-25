from Map import Map
from Circle import Circle
from Line import Line

class TestMap(Map):
  '''
  Hard-coded course map
  '''
  # TODO: Draw ASCII Map Here

  # TODO: Measurements in feet. Convert to meters. Degrees, need radians

  #--------------------------------------------------------
  #                    CIRCLES
  #--------------------------------------------------------


  #--------------------------------------------------------
  #                    LINES
  #--------------------------------------------------------
 
  # Test Constants
  T_START_POINT = [ -5.0, -18.5 ]
  T_END_POINT = [ 5.0, -18.5 ]

  #-------------------------------------------------------------------------------
  def __init__(self):
    '''
    Initializes the course map
    '''
    super(TestMap, self).__init__()
    # Course Circles
    self.circles = []
    # Course Lines
    self.tl = Line(TestMap.T_START_POINT, TestMap.T_END_POINT)
    self.lines = [self.tl]

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
    desc = "Map:\n"
    desc += "\tTL: {0}\n".format(self.tl)
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

