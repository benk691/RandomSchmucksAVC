import math
import time
import Constants
from threading import Thread
from ParticleFilter import ParticleFilter
from Publisher import Publisher
from GeneralFunctions import rotate

class ControlPlanner(Thread, Publisher):
  '''
  Determines what action to take to get the vehicle to where we want to go.
  '''
  #-------------------------------------------------------------------------------
  def __init__(self, group=None, target=None, name=None, args=(), kwargs={}, daemon=None, courseMap=None, sensorConversionThread=None):
    '''
    Initializes the control planner
    @param Refer to the Python Thread class for documentation on all thread specific parameters
    @param courseMap - the course map
    @param sensorConversionThread - the sensor conversion thread
    '''
    Thread.__init__(self, group=group, target=target, name=name, daemon=daemon)
    Publisher.__init__(self)
    self.args = args
    self.kwargs = kwargs
    self.courseMap = courseMap
    self.particleFilter = ParticleFilter(Constants.PARTICLE_NUMBER, Constants.MAP_START_BOX, Constants.MAP_HEADING_RANGE, self.courseMap, sensorConversionThread)
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
      sleepTime = (1.0 / Constants.CONTROL_UPDATE_RATE) - (time.time() - self.particleFilter.currentTime)
      if sleepTime > Constants.CONTROL_SLEEP_THRESHOLD:
        time.sleep(sleepTime)
      # TODO: Modify number of particles

  #-------------------------------------------------------------------------------
  def shutdown(self):
    '''
    Shutdown the thread
    '''
    self.shutDown = True

  #-------------------------------------------------------------------------------
  def _control(self):
    '''
    Send control to vehicle
    '''
    xErr = self.courseMap.waypoints[self.waypoint][Constants.X] - self.estVehicleX
    yErr = self.courseMap.waypoints[self.waypoint][Constants.Y] - self.estVehicleY
    xErr, yErr = rotate(xErr, yErr, -self.estVehicleHeading)

    self.steeringAngleGoal = math.atan(Constants.VEHICLE_AXLE_LEN * ((2.0 * yErr) / (math.pow(xErr, 2) + math.pow(yErr, 2)))) * Constants.CONTROL_STEERING_AGRESSION
    self.velocityGoal = max(Constants.MIN_VEHICLE_VELOCITY, Constants.MAX_VEHICLE_VELOCITY - math.atan(self.steeringAngleGoal) * Constants.VELOCITY_SCALE_FACTOR)
    # Publish goals
    self.publish(self.velocityGoal, self.steeringAngleGoal)

  #------------------------------------------------------------------------------- 
  def _checkWaypoint(self):
    '''
    Checks if the estimated vehicle location has passed the current waypoint goal.
    If the vehicle has then increment to the next waypoint
    '''
    #print("DBG: waypoint = {0}".format(self.waypoint))
    #print("DBG: len(courseMap.waypoints) = {0}".format(len(self.courseMap.waypoints)))
    #print("DBG: courseMap.waypoints:")
    #for wp in self.courseMap.waypoints:
    #  print("DBG: wp = {0}".format(wp))

    rWpX, rWpY = rotate(self.courseMap.waypoints[self.waypoint][Constants.X], self.courseMap.waypoints[self.waypoint][Constants.Y], -self.courseMap.waypoints[self.waypoint][Constants.HEADING])
    rEstX, rEstY = rotate(self.estVehicleX, self.estVehicleY, -self.courseMap.waypoints[self.waypoint][Constants.HEADING])
    if rEstX > (rWpX - Constants.WAYPOINT_CHECK_DIST):
      self.waypointCheck += 1
    else:
      self.waypointCheck = 0

    if self.waypointCheck >= Constants.WAYPOINT_MAX_CHECKS:
      self.waypointCheck = 0
      self.waypoint += 1
      if self.waypoint > len(self.courseMap.waypoints):
        self.waypoint = 0
      # TODO: Add in switch cases for special points (NERF, stop, etc...). IDEA: Add a call back to the waypoint list, if it has a callback then call it otherwise no special function

  #-------------------------------------------------------------------------------
  def _debugDescription(self):
    '''
    Generates debugging information about the control planner
    @return string describing debug information
    '''
    desc = "ControlPlanner:\n"
    # TODO: add in setTabs for particle filter and course map
    desc += "\tshutDown = {0}\n".format(self.shutDown)
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

