from Map import Map
from Circle import Circle
from Line import Line

class CourseMap(Map):
  '''
  Hard-coded course map
  '''
  # TODO: Draw ASCII Map Here

  # TODO: Degrees, need radians

  #--------------------------------------------------------
  #                    CIRCLES
  #--------------------------------------------------------

  # IQ1C : Inner Quadrant 1 Circle
  # OQ1C : Outer Quadrant 1 Circle
  # IQ3C : Inner Quadrant 3 Circle
  # OQ3C : Outer Quadrant 3 Circle

  # IQ1C Constants
  IQ1C_CENTER = [ 31.0 / 0.254, 31.0 / 0.254 ] 
  IQ1C_RADIUS = 23.0 / 0.254
  IQ1C_START_ANGLE = -90.0
  IQ1C_STOP_ANGLE = 180.0

  # OQ1C Constants
  OQ1C_CENTER = [ 31.0 / 0.254, 31.0 / 0.254 ]
  OQ1C_RADIUS = 39.0 / 0.254
  OQ1C_START_ANGLE = -90.0
  OQ1C_STOP_ANGLE = 180.0

  # IQ3C Constants
  IQ3C_CENTER = [ -31.0 / 0.254, -31.0 / 0.254 ]
  IQ3C_RADIUS = 23.0 / 0.254
  IQ3C_START_ANGLE = 90.0
  IQ3C_STOP_ANGLE = 360.0

  # OQ3C Constants
  OQ3C_CENTER = [ -31.0 / 0.254, -31.0 / 0.254 ]
  OQ3C_RADIUS = 39.0 / 0.254
  OQ3C_START_ANGLE = 90.0
  OQ3C_STOP_ANGLE = 360.0

  #--------------------------------------------------------
  #                    LINES
  #--------------------------------------------------------
 
  # IQ1XL : Inner Quadrant 1 X Parallel Line
  # IQ1YL : Inner Quadrant 1 Y Parallel Line
  # OQ2XL : Outer Quadrant 2 X Parallel Line
  # OQ2YL : Outer Quadrant 2 Y Parallel Line
  # IQ3XL : Inner Quadrant 3 X Parallel Line
  # IQ3YL : Inner Quadrant 3 Y Parallel Line
  # OQ4XL : Outer Quadrant 4 X Parallel Line
  # OQ4YL : Outer Quadrant 4 Y Parallel Line

  # IQ1XL Constants
  IQ1XL_START_POINT = [ 8.0 / 0.254, 8.0 / 0.254 ]
  IQ1XL_END_POINT = [ 31.0 / 0.254, 8.0 / 0.254 ]

  # IQ1YL Constants
  IQ1YL_START_POINT = [ 8.0 / 0.254, 8.0 / 0.254 ]
  IQ1YL_END_POINT = [ 8.0 / 0.254, 31.0 / 0.254 ]

  # OQ2XL Constants
  OQ2XL_START_POINT = [ -8.0 / 0.254, 8.0 / 0.254 ]
  OQ2XL_END_POINT = [ -31.0 / 0.254, 8.0 / 0.254 ]

  # OQ2YL Constants
  OQ2YL_START_POINT = [ -8.0 / 0.254, 8.0 / 0.254 ]
  OQ2YL_END_POINT = [ -8.0 / 0.254, 31.0 / 0.254 ]

  # IQ3XL Constants
  IQ3XL_START_POINT = [ -8.0 / 0.254, -8.0 / 0.254 ]
  IQ3XL_END_POINT = [ -31.0 / 0.254, -8.0 / 0.254 ]

  # IQ3YL Constants
  IQ3YL_START_POINT = [ -8.0 / 0.254, -8.0 / 0.254 ]
  IQ3YL_END_POINT = [ -8.0 / 0.254, -31.0 / 0.254 ]

  # OQ4XL Constants
  OQ4XL_START_POINT = [ 8.0 / 0.254, -8.0 / 0.254 ]
  OQ4XL_END_POINT = [ 31.0 / 0.254, -8.0 / 0.254 ]

  # OQ4YL Constants
  OQ4YL_START_POINT = [ 8.0 / 0.254, -8.0 / 0.254 ]
  OQ4YL_END_POINT = [ 8.0 / 0.254, -31.0 / 0.254 ]

  #-------------------------------------------------------------------------------
  def __init__(self):
    '''
    Initializes the course map
    '''
    super(CourseMap, self).__init__()
    # Course Circles
    self.iq1c = Circle(Map.IQ1C_CENTER, Map.IQ1C_RADIUS, Map.IQ1C_START_ANGLE, Map.IQ1C_STOP_ANGLE)
    self.oq1c = Circle(Map.OQ1C_CENTER, Map.OQ1C_RADIUS, Map.OQ1C_START_ANGLE, Map.OQ1C_STOP_ANGLE)
    self.iq3c = Circle(Map.IQ3C_CENTER, Map.IQ3C_RADIUS, Map.IQ3C_START_ANGLE, Map.IQ3C_STOP_ANGLE)
    self.oq3c = Circle(Map.OQ3C_CENTER, Map.OQ3C_RADIUS, Map.OQ3C_START_ANGLE, Map.OQ3C_STOP_ANGLE)
    self.circles = [self.iq1c, self.oq1c, self.iq3c, self.oq3c]
    # Course Lines
    self.iq1xl = Line(Map.IQ1XL_START_POINT, Map.IQ1XL_END_POINT)
    self.iq1yl = Line(Map.IQ1YL_START_POINT, Map.IQ1YL_END_POINT)
    self.oq2xl = Line(Map.OQ2XL_START_POINT, Map.OQ2XL_END_POINT)
    self.oq2yl = Line(Map.OQ2YL_START_POINT, Map.OQ2YL_END_POINT)
    self.iq3xl = Line(Map.IQ3XL_START_POINT, Map.IQ3XL_END_POINT)
    self.iq3yl = Line(Map.IQ3YL_START_POINT, Map.IQ3YL_END_POINT)
    self.oq4xl = Line(Map.OQ4XL_START_POINT, Map.OQ4XL_END_POINT)
    self.oq4yl = Line(Map.OQ4YL_START_POINT, Map.OQ4YL_END_POINT)
    self.lines = [self.iq1xl, self.iq1yl, self.oq2xl, self.oq2yl, self.iq3xl, self.iq3yl, self.oq4xl, self.oq4yl]

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
    desc += "\tIQ1C: {0}\n".format(self.iq1c)
    desc += "\tOQ1C: {0}\n".format(self.oq1c)
    desc += "\tIQ3C: {0}\n".format(self.iq3c)
    desc += "\tOQ3C: {0}\n".format(self.oq3c)
    desc += "\tIQ1XL: {0}\n".format(self.iq1xl)
    desc += "\tIQ1YL: {0}\n".format(self.iq1yl)
    desc += "\tOQ2XL: {0}\n".format(self.oq2xl)
    desc += "\tOQ2YL: {0}\n".format(self.oq2yl)
    desc += "\tIQ3XL: {0}\n".format(self.iq3xl)
    desc += "\tIQ3YL: {0}\n".format(self.iq3yl)
    desc += "\tOQ4XL: {0}\n".format(self.oq4xl)
    desc += "\tOQ4YL: {0}\n".format(self.oq4yl)
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

