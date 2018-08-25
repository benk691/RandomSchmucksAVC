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
  T1_START_POINT = [ -5.0, -17.0 ]
  T1_END_POINT = [ 50.0, -17.0 ]

  T2_START_POINT = [ -5.0, -23.0 ]
  T2_END_POINT = [ 50.0, -23.0 ]

  #-------------------------------------------------------------------------------
  def __init__(self):
    '''
    Initializes the course map
    '''
    super(TestMap, self).__init__()
    # Course Circles
    self.circles = []
    # Course Lines
    self.t1 = Line(TestMap.T1_START_POINT, TestMap.T1_END_POINT)
    self.t2 = Line(TestMap.T2_START_POINT, TestMap.T2_END_POINT)
    self.lines = [self.t1, self.t2]

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
    desc += "\tT1: {0}\n".format(self.t1)
    desc += "\tT2: {0}\n".format(self.t2)
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

