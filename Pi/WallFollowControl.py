import time
import Constants
from threading import Thread
from Publisher import Publisher
from PID import PID

class WallFollowControl(Thread, Publisher):
  '''
  Contingency control plan to have the vehicle follow the walls of the course
  '''
  #-------------------------------------------------------------------------------
  def __init__(self, group=None, target=None, name=None, args=(), kwargs={}, daemon=None, sensorConversionThread=None):
    '''
    Initializes the control planner
    @param Refer to the Python Thread class for documentation on all thread specific parameters
    @param sensorConversionThread - the sensor conversion thread
    '''
    Thread.__init__(self, group=group, target=target, name=name, daemon=daemon)
    Publisher.__init__(self)
    self.args = args
    self.kwargs = kwargs
    self.sensorConversionThread = sensorConversionThread
    self.shutDown = False
    self.wallFollowPID = PID(Constants.WALL_FOLLOW_PID_P, Constants.WALL_FOLLOW_PID_I, Constants.WALL_FOLLOW_PID_D, Constants.WALL_FOLLOW_PID_WINDUP)
    self.wallFollowPID.setGoal(Constants.WALL_FOLLOW_PID_DIST_GOAL)
    self.vehicleLeftDistance = 0.0
    self.vehicleRightDistance = 0.0
    self.vehicleSteeringAngle = 0.0
    self.steeringAngleGoal = 0.0
    # TODO: Velocity goals?
    self.velocityGoal = 0.5
    self.currentTime = time.time()

  #-------------------------------------------------------------------------------
  def run(self):
    '''
    Runs the planner to set goals for where the vehicle is going to go
    '''
    while not self.shutDown:
      self.currentTime = time.time()
      self._getVehicleMeasurements()
      self._control()

      sleepTime = (1.0 / Constants.WALL_FOLLOW_UPDATE_RATE) - (time.time() - self.currentTime)
      if sleepTime > Constants.CONTROL_SLEEP_THRESHOLD:
        time.sleep(sleepTime)

  #-------------------------------------------------------------------------------
  def shutdown(self):
    '''
    Shutdown the thread
    '''
    self.shutDown = True

  #-------------------------------------------------------------------------------
  def _control(self):
    '''
    Sets the control goals for the vehicle
    '''
    # Publish goals
    # TODO: Convert to angle then add STRAIGHT?
    self.steeringAngleGoal = (self.wallFollowPID.control() + Constants.POT_STRAIGHT)
    self.publish(self.velocityGoal, self.steeringAngleGoal)

  #-------------------------------------------------------------------------------
  def _getVehicleMeasurements(self):
    '''
    Read the sensor measurements from the vehicle
    '''
    self.vehicleLeftDistance = self.sensorConversionThread.leftDistance
    self.vehicleRightDistance = self.sensorConversionThread.rightDistance
    self.vehicleSteeringAngle = self.sensorConversionThread.steeringAngle
    # TODO: Use left wall?
    self.wallFollowPID.setMeasurement(self.vehicleLeftDistance)

  #-------------------------------------------------------------------------------
  def __del__(self):
    '''
    Destructor
    '''
    self.join(timeout=5)

