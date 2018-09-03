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
    super(ControlPlanner, self).__init__(group=group, target=target, name=name, daemon=daemon)
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

    if self.vehicle is not None:
      self.vehicle.setTabs(1)

  #-------------------------------------------------------------------------------
  def run(self):
    '''
    Runs the planner to set goals for where the vehicle is going to go
    '''
    while not self.shutDown:
      #print("DBG: R0")
      self.estVehicleX, self.estVehicleY, self.estVehicleHeading, self.covarVehicle = self.particleFilter.getEstiamtedVehicleLocation()
      #print("DBG: R1")
      self._checkWaypoint()
      #print("DBG: R2")
      self._control()
      #print("DBG: R3")
      self.particleFilter.prevTime = self.particleFilter.currentTime
      #print("DBG: R4")
      sleepTime = (1.0 / Constants.CONTROL_UPDATE_RATE) - (time.time() - self.particleFilter.currentTime)
      #print("DBG: R5")
      if sleepTime > Constants.CONTROL_SLEEP_THRESHOLD:
        time.sleep(sleepTime)
      #print("DBG: R7")
      # TODO: Modify number of particles

  #-------------------------------------------------------------------------------
  def _control(self):
    '''
    Send control to vehicle
    '''
    xErr = self.courseMap.waypoints[self.waypoint][Constants.X] - self.estVehicleX
    yErr = self.courseMap.waypoints[self.waypoint][Constants.Y] - self.estVehicleY
    xErr, yErr = self.particleFilter._rotate(xErr, yErr, -self.estVehicleHeading)

    self.steeringAngleGoal = math.atan(Constants.VEHICLE_AXLE_LEN * ((2.0 * yErr) / (math.pow(xErr, 2) + math.pow(yErr, 2)))) * Constants.CONTROL_STEERING_AGRESSION
    self.velocityGoal = max(Constants.MIN_VEHICLE_VELOCITY, Constants.MAX_VEHICLE_VELOCITY - math.atan(self.steeringAngleGoal) * Constants.VELOCITY_SCALE_FACTOR)

  #------------------------------------------------------------------------------- 
  def _checkWaypoint(self):
    '''
    Checks if the estimated vehicle location has passed the current waypoint goal.
    If the vehicle has then increment to the next waypoint
    '''
    # TODO: Pull rotate out into general functions
    #print("DBG: W0")
    print("DBG: waypoint = {0}".format(self.waypoint))
    print("DBG: len(courseMap.waypoints) = {0}".format(len(self.courseMap.waypoints)))
    print("DBG: courseMap.waypoints:")
    for wp in self.courseMap.waypoints:
      print("DBG: wp = {0}".format(wp))

    rWpX, rWpY = self.particleFilter._rotate(self.courseMap.waypoints[self.waypoint][Constants.X], self.courseMap.waypoints[self.waypoint][Constants.Y], -self.courseMap.waypoints[self.waypoint][Constants.HEADING])
    #print("DBG: W1")
    rEstX, rEstY = self.particleFilter._rotate(self.estVehicleX, self.estVehicleY, -self.courseMap.waypoints[self.waypoint][Constants.HEADING])
    #print("DBG: W2")
    if rEstX > (rWpX - Constants.WAYPOINT_CHECK_DIST):
      self.waypointCheck += 1
    else:
      self.waypointCheck = 0
    #print("DBG: W3")

    if self.waypointCheck >= Constants.WAYPOINT_MAX_CHECKS:
      self.waypointCheck = 0
      self.waypoint += 1
      if self.waypoint > len(self.courseMap.waypoints):
        self.waypoint = 0
      # TODO: Add in switch cases for special points (NERF, stop, etc...)

    #print("DBG: W4")

  #-------------------------------------------------------------------------------
  def _debugDescription(self):
    '''
    Generates debugging information about the control planner
    @return string describing debug information
    '''
    desc = "ControlPlanner:\n"
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
    desc += "\tsteeringAngleGoal = {0}\n".format(self.steeringAngleGoal)
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
