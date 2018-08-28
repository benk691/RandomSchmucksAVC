from threading import Thread

class ControlPlanner(Thread):
  '''
  Determines what action to take to get the vehicle to where we want to go.
  '''
  #-------------------------------------------------------------------------------
  def __init__(self, group=None, target=None, name=None, args=(), kwargs={}, daemon=None, vehicle=None, particleFilter=None, courseMap=None):
    '''
    Initializes the control planner
    @param Refer to the Python Thread class for documentation on all thread specific parameters
    @param vehicle - the real vehicle to control
    @param particleFilter - the particle filter used for estimating where the vehicle is
    @param courseMap - the course map
    '''
    self.vehicle = vehicle
    self.particleFilter = particleFilter
    self.courseMap = courseMap
    self.waypoints = []
    self.shutDown = False
    self.estVehicleX = 0.0
    self.estVehicleY = 0.0
    self.estVehicleHeading = 0.0
    self.covarVehicle = None

  #-------------------------------------------------------------------------------
  def run(self):
    '''
    Runs the planner to set goals for where the vehicle is going to go
    '''
    while not self.shutDown:
      self.estVehicleX, self.estVehicleY, self.estVehicleHeading, self.covarVehicle = self.particleFilter.getEstiamtedVehicleLocation()

  #-------------------------------------------------------------------------------
  def _debugDescription(self):
    '''
    Generates debugging information about the control planner
    @return string describing debug information
    '''
    desc = "ControlPlanner:\n"
    vehicle.setTabs(1)
    # TODO: add in setTabs for particle filter and course map
    desc += "\tshutDown = {0}\n".format(self.shutDown)
    desc += "\tvehicle = {0}\n".format(self.vehicle)
    desc += "\tparticleFilter = {0}\n".format(self.particleFilter)
    desc += "\tcourseMap = {0}\n".format(self.courseMap)
    desc += "\testVehicleX = {0}\n".format(self.estVehicleX)
    desc += "\testVehicleY = {0}\n".format(self.estVehicleY)
    desc += "\testVehicleHeading = {0}\n".format(self.estVehicleHeading)
    desc += "\tcovarVehicle = {0}\n".format(self.covarVehicle)
    desc += "\twaypoints:\n"
    for wp in self.waypoints
      desc += "\t\t{0}\n".format(wp)
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

  #-------------------------------------------------------------------------------
  def __del__(self):
    '''
    Destructor
    '''
    self.join(timeout=5)

