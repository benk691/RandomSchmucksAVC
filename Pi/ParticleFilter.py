import random
import math
import time
from scipy.stats import norm
from Constants import Constants

class ParticleFilter:
  #-------------------------------------------------------------------------------
  def __init__(self, particleNumber, startBox, headingRange):
    '''
    Initializes the particle filter
    @param particleNumber - number of particles to generate
    @param startBox - the inital box to create particles in. List of two X, Y coordinates defining the corners of the box
    @param headingRange - the range of headings to create particles with. List of two doubles.
    '''
    random.seed()
    self.particleNumber = particleNumber
    self.prevTime = time.time()
    self.currentTime = time.time()
    self.dt = 0.0
    self.vehicleVelocity = 0.0
    self.vehicleHeading = 0.0
    self.vehicleLeftDistance = 0.0
    self.vehicleRightDistance = 0.0
    self.vehicleSteeringAngle = 0.0
    self._maxSteeringAngle = max(Constants.MAX_LEFT_STEERING_ANGLE, Constants.MAX_RIGHT_STEERING_ANGLE)
    self._minSteeringAngle = min(Constants.MAX_LEFT_STEERING_ANGLE, Constants.MAX_RIGHT_STEERING_ANGLE)
    # Particle index: [x, y, heading, weight]
    self.particles = [ [random.uniform(startBox[0][Constants.X], startBox[1][Constants.X]), random.uniform(startBox[0][Constants.Y], startBox[1][Constants.Y]), random.uniform(headingRange[0], headingRange[1]), 1.0] for i in range(self.particleNumber) ]

  #-------------------------------------------------------------------------------
  def run(self):
    '''
    Run the praticle filter
    '''
    while True:
      self._getVehicleMeasurements()
      self._predict()
      self._weight()

  #-------------------------------------------------------------------------------
  def _predict(self):
    '''
    Perform the prediction step. This will predict where the particles are going to go 
    based on the most recent values read in from the vehicle
    '''
    self.dt = self.currentTime - self.prevTime
    for i in range(self.particleNumber):
      # Generate a steering angle for the particles
      genSteeringAngle = random.guass(mu=self.vehicleSteeringAngle, sigma=Constants.STEERING_ANGLE_NOISE)
      if genSteeringAngle > self._maxSteeringAngle:
        genSteeringAngle = self._maxSteeringAngle
      if genSteeringAngle < selg._minSteeringAngle:
        genSteeringAngle = self,_minSteeringAngle
      turnRadius = Constants.VEHICLE_AXLE_LEN / math.tan(genSteeringAngle)
      # Generate a velocity for the particles
      genVelocity = random.guass(mu=self.vehicleVelocity, sigma=Constants.VELOCITY_NOISE) * self.dt
      rotatedX, rotatedY = self._rotate(turnRadius * math.sin(genVelocity / turnRadius), -turnRadius * (1 - math.cos(genVelocity / turnRadius)), self.particles[i][Constants.HEADING])
      self.particles[i][Constants.X] += rotatedX
      self.particles[i][Constants.Y] += rotatedY
      self.particles[i][Constants.HEADING] += genVelocity / turnRadius

  #-------------------------------------------------------------------------------
  def _weight(self):
    '''
    Calculates the weights for each particle
    '''
    for i in range(self.particleNumber):
      # TODO: Calculate distance line of sight and intersections
      particleDistLeft = 0.0
      particleDistRight = 0.0
      # PDF(measurement, mean, std_dev)
      self.particles[i][Constants.WEIGHT] *= norm.pdf(self.particles[i][HEADING], self.vehicleHeading, Constants.HEADING_NOISE)
      self.particles[i][Constants.WEIGHT] *= norm.pdf(particleDistLeft, self.vehicleLeftDistance, Constants.DISTANCE_NOISE)
      self.particles[i][Constants.WEIGHT] *= norm.pdf(particleDistRight, self.vehicleRightDistance, Constants.DISTANCE_NOISE)

  #-------------------------------------------------------------------------------
  def measure(self):
    pass

  #-------------------------------------------------------------------------------
  def normalizeWeights(self):
    pass

  #-------------------------------------------------------------------------------
  def checksumParticles(self):
    pass

  #-------------------------------------------------------------------------------
  def distributeNewParticles(self):
    pass

  #-------------------------------------------------------------------------------
  def getEstiamtedLocation(self):
    pass

  #-------------------------------------------------------------------------------
  def _rotate(self, x, y, theta):
    '''
    Rotates the X, Y coordinate by theta
    x - the X coordinate
    y - the Y coordinate
    theta - the amount to rotate (radians)
    '''
    # TODO: Implement rotation
    return x, y

  #-------------------------------------------------------------------------------
  def _getVehicleMeasurements(self):
    '''
    Read the sensor measurements from the vehicle
    '''
    # TODO: Connect this with the sensor conversion/filtered reading thread
    self.currentTime = time.time()
    self.vehicleVelocity = 0.0
    self.vehicleHeading = 0.0
    self.vehicleLeftDistance = 0.0
    self.vehicleRightDistance = 0.0
    self.vehicleSteeringAngle = 0.0

  #-------------------------------------------------------------------------------
  def _debugDescription(self):
    '''
    Generates debugging information about the particle filter
    @return string describing debug information
    '''
    desc = "Particle Filter:\n"
    desc += "\tparticleNumber = {0}\n".format(self.particleNumber)
    desc += "\tprevTime = {0}\n".format(self.prevTime)
    desc += "\tcurrentTime = {0}\n".format(self.currentTime)
    desc += "\tdt = {0}\n".format(self.dt)
    desc += "\tvehicleVelocity = {0}\n".format(self.vehicleVelocity)
    desc += "\tvehicleHeading = {0}\n".format(self.vehicleHeading)
    desc += "\tvehicleLeftDistance = {0}\n".format(self.vehicleLeftDistance)
    desc += "\tvehicleRightDistance = {0}\n".format(self.vehicleRightDistance)
    desc += "\tvehicleSteeringAngle = {0}\n".format(self.vehicleSteeringAngle)
    desc += "\tparticles:\n"
    for p in self.particles:
      desc += "\t\t[X = {0}, Y = {1}, H = {2}, W = {3}]\n".format(p[0], p[1], p[2], p[3])
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

