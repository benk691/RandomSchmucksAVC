import math
import time
from threading import Thread
import Constants

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
    self.shutDown = False
    self.estVehicleX = 0.0
    self.estVehicleY = 0.0
    self.estVehicleHeading = 0.0
    self.covarVehicle = None
    self.waypoint = 0
    self.waypointCheck = 0
    self.steeringAngleGoal = 0.0
    self.velocityGoal = 0.0

  #-------------------------------------------------------------------------------
  def run(self):
    '''
    Runs the planner to set goals for where the vehicle is going to go
    '''
    while not self.shutDown:
      self.estVehicleX, self.estVehicleY, self.estVehicleHeading, self.covarVehicle = self.particleFilter.getEstiamtedVehicleLocation()
      self._checkWaypoint()
      self._control()
      self.particleFilter.prevTime = self.particleFilter.currentTime
      sleepTime = (1.0 /Constants.CONTROL_UPDATE_RATE) - (time.time() - self.particleFilter.currentTime)
      if sleepTime > Constants.CONTROL_SLEEP_THRESHOLD:
        time.sleep(sleepTime)
      # TODO: Modify number of particles

  #-------------------------------------------------------------------------------
  def _control(self):
    '''
    Send control to vehicle
    '''
    xErr = self.courseMap.waypoints[self.waypoint][Constants.X] - self.estVehicleX
    yErr = self.courseMap.waypoints[self.waypoint][Constants.Y] - self.estVehicleY
    xErr, yErr = self.particleFilter._rotate(xErr, yErr, -self.estVehicleHeading)

    self.steeringAngleGoal = math.arctan(yErr / xErr)
    self.velocityGoal = max(Constants.MIN_VEHICLE_VELOCITY, Constants.MAX_VEHICLE_VELOCITY - math.arctan(steeringAngleGoal) * Constants.VELOCITY_SCALE_FACTOR)

  #------------------------------------------------------------------------------- 
  def _checkWaypoint(self):
    '''
    Checks if the estimated vehicle location has passed the current waypoint goal.
    If the vehicle has then increment to the next waypoint
    '''
    # TODO: Pull rotate out into general functions
    rWpX, rWpY = self.particleFilter._rotate(self.courseMap.waypoints[self.waypoint][Constants.X], self.courseMap.waypoints[self.waypoint][Constants.Y], -self.courseMap.waypoints[self.waypoint][Constants.HEADING])
    rEstX, rEstY = self.particleFilter._rotate(self.estVehicleX, self.estVehicleY, -self.courseMap.waypoints[self.waypoint][Constants.HEADING])
    if rEstX > (rWpX - Constants.WAYPOINT_CHECK_DIST):
      self.waypointCheck += 1
    else:
      self.waypointCheck = 0

    if self.waypointCheck >= Constants.WAYPOINT_MAX_CHECKS:
      self.waypointCheck = 0
      self.waypoint += 1
      if self.waypoint > len(self.courseMap.waypoints):
        self.waypoint = 0
      # TODO: Add in switch cases for special points (NERF, stop, etc...)

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
    desc += "\twaypoint = [{0}: {1}]\n".format(self.waypoint, self.courseMap.waypoints[self.waypoint])
    desc += "\twaypointCheck = {0}\n".format(self.waypointCheck)
    desc += "\tvelocityGoal = {0}\n".format(self.velocityGoal)
    desc += "\tsteeringGoal = {0}\n".format(self.steeringGoal)
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

