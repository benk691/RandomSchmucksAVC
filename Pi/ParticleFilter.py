import random
import math
import time
import numpy
import Constants
from scipy.stats import norm
from Line import Line
from GeneralFunctions import rotate

class ParticleFilter:
  #-------------------------------------------------------------------------------
  def __init__(self, particleNumber, startBox, headingRange, courseMap):
    '''
    Initializes the particle filter
    @param particleNumber - number of particles to generate
    @param startBox - the inital box to create particles in. List of two X, Y coordinates defining the corners of the box
    @param headingRange - the range of headings to create particles with. List of two doubles.
    @param courseMap - the course map
    '''
    random.seed()
    self.prevTime = time.time()
    self.currentTime = time.time()
    self.courseMap = courseMap
    self.dt = 0.0
    self.vehiclePrevTotalStripCount = 0.0
    self.vehicleTotalStripCount = 0.0
    self.vehicleVelocity = 0.0
    self.vehicleHeading = 0.0
    self.vehicleLeftDistance = 0.0
    self.vehicleRightDistance = 0.0
    self.vehicleSteeringAngle = 0.0
    self.estVehicleX = 0.0
    self.estVehicleY = 0.0
    self.estVehicleHeading = 0.0
    self.covarVehicle = []
    self.cumulativeSum = []
    self._maxSteeringAngle = max(Constants.MAX_LEFT_STEERING_ANGLE, Constants.MAX_RIGHT_STEERING_ANGLE)
    self._minSteeringAngle = min(Constants.MAX_LEFT_STEERING_ANGLE, Constants.MAX_RIGHT_STEERING_ANGLE)
    # Particle index: [x, y, heading, weight]
    self.particles = [ [random.uniform(startBox[0][Constants.X], startBox[1][Constants.X]), random.uniform(startBox[0][Constants.Y], startBox[1][Constants.Y]), random.uniform(headingRange[0], headingRange[1]), 1.0, 0.0, 0.0] for i in range(particleNumber) ]

  #-------------------------------------------------------------------------------
  def getEstiamtedVehicleLocation(self, totalStripCount, heading, leftDistance, rightDistance, steeringAngle):
    '''
    Gets the estimated vehicle location based on the particles
    @return mean(X), mean(Y), mean(heading), covariance
    '''
    self.currentTime = time.time()
    self._getVehicleMeasurements(totalStripCount, heading, leftDistance, rightDistance, steeringAngle)
    self.dt = self.currentTime - self.prevTime
    # Perform particle calculations
    self._predict()
    self._weight()
    self._generateNewParticleList()
    # Calculate estimates
    self.estVehicleX = sum([ p[Constants.X] for p in self.particles]) / float(len(self.particles))

    self.estVehicleY = sum([ p[Constants.Y] for p in self.particles]) / float(len(self.particles))
    self.estVehicleHeading = sum([ p[Constants.HEADING] for p in self.particles]) / float(len(self.particles))
    # Calculate covariance
    x = [ p[Constants.X] for p in self.particles ]
    y = [ p[Constants.Y] for p in self.particles ]
    h = [ p[Constants.HEADING] for p in self.particles ]
    self.covarVehicle = numpy.cov(numpy.vstack((x,y,h)))
    return self.estVehicleX, self.estVehicleY, self.estVehicleHeading, self.covarVehicle

  #-------------------------------------------------------------------------------
  def _getVehicleMeasurements(self, totalStripCount, heading, leftDistance, rightDistance, steeringAngle):
    '''
    Read the sensor measurements from the vehicle
    '''
    self.vehicleTotalStripCount = totalStripCount
    self.vehicleHeading = heading
    self.vehicleLeftDistance = leftDistance
    self.vehicleRightDistance = rightDistance
    self.vehicleSteeringAngle = steeringAngle
    if self.dt == 0.0:
      self.dt = 0.00000001
    self.vehicleVelocity = (self.vehicleTotalStripCount - self.vehiclePrevTotalStripCount) / self.dt * Constants.STRIP_COUNT_TO_METERS
    self.vehiclePrevTotalStripCount = self.vehicleTotalStripCount
    # TODO: This is in the sensor conversion thread, still need?
    if self.vehicleLeftDistance >= Constants.DIST_MAX_DISTANCE:
      self.vehicleLeftDistance = Constants.DIST_MAX_DISTANCE + 2.0 * Constants.DISTANCE_NOISE

    if self.vehicleRightDistance >= Constants.DIST_MAX_DISTANCE:
      self.vehicleRightDistance = Constants.DIST_MAX_DISTANCE + 2.0 * Constants.DISTANCE_NOISE

  #-------------------------------------------------------------------------------
  def _predict(self):
    '''
    Perform the prediction step. This will predict where the particle is going to go 
    based on the most recent values read in from the vehicle
    '''
    for i in range(len(self.particles)):
      # Generate a steering angle for the particles
      genSteeringAngle = random.gauss(mu=self.vehicleSteeringAngle, sigma=Constants.STEERING_ANGLE_NOISE)
      if genSteeringAngle > self._maxSteeringAngle:
        genSteeringAngle = self._maxSteeringAngle
      if genSteeringAngle < self._minSteeringAngle:
        genSteeringAngle = self._minSteeringAngle
      # TODO: Check solution to division of zero with Brian
      turnRadius = 10000000.0
      if math.tan(genSteeringAngle) != 0.0:
        turnRadius = Constants.VEHICLE_AXLE_LEN / math.tan(genSteeringAngle)
      if turnRadius == 0.0:
        turnRadius = 0.000000001
      # Generate a velocity for the particles
      genVelocity = random.gauss(mu=self.vehicleVelocity, sigma=Constants.VELOCITY_NOISE) * self.dt
      rotatedX, rotatedY = rotate(turnRadius * math.sin(genVelocity / turnRadius), turnRadius * (1 - math.cos(genVelocity / turnRadius)), self.particles[i][Constants.HEADING])
      #print("DBG: genSteeringAngle = {0}".format(genSteeringAngle))
      #print("DBG: turnRadius = {0}".format(turnRadius))
      #print("DBG: genVelocity = {0}".format(genVelocity))
      #print("DBG: rotatedX, rotatedY = ({0}, {1})\n".format(rotatedX, rotatedY))
      slipX, slipY = rotate(0, random.gauss(0, Constants.SLIP_NOISE * self.dt), self.particles[i][Constants.HEADING])
      self.particles[i][Constants.X] += rotatedX + slipX
      self.particles[i][Constants.Y] += rotatedY + slipY
      self.particles[i][Constants.HEADING] += genVelocity / turnRadius

  #-------------------------------------------------------------------------------
  def _weight(self):
    '''
    Calculates the weights for the given particle
    '''
    for i in range(len(self.particles)):
      #print("DBG: vehicleHeading = {0}".format(self.vehicleHeading))
      particleDistLeft, particleDistRight = self._calculateDistanceLineOfSight(self.particles[i])
      #print("DBG: particleDistLeft = {0}".format(particleDistLeft))
      #print("DBG: particleDistRight = {0}".format(particleDistRight))
      # PDF(measurement, mean, std_dev)
      #print("DBG: self.particles[{0}][Constants.HEADING] = {1}".format(i, self.particles[i][Constants.HEADING]))
      #print("DBG: self.vehicleHeading = {0}".format(self.vehicleHeading))
      #print("DBG: Constants.HEADING_NOISE = {0}".format(Constants.HEADING_NOISE))
      headingPDF = norm.pdf(self.particles[i][Constants.HEADING], self.vehicleHeading, Constants.HEADING_NOISE)
      # Adding a fraction times the peak probability
      # Chances that the measurment is bad
      self.particles[i][Constants.WEIGHT] *= headingPDF + 0.03 * 7.62

      #print("DBG: particleDistLeft = {0}".format(particleDistLeft))
      #print("DBG: self.vehicleLeftDistance = {0}".format(self.vehicleLeftDistance))
      #print("DBG: Constants.DISTANCE_NOISE = {0}".format(Constants.DISTANCE_NOISE))
      leftDistPDF = norm.pdf(particleDistLeft, self.vehicleLeftDistance, Constants.DISTANCE_NOISE)
      # Adding a fraction times the peak probability
      # Chances that the measurment is bad
      self.particles[i][Constants.WEIGHT] *= leftDistPDF + 0.003 * 1.33

      #print("DBG: particleDistRight = {0}".format(particleDistRight))
      #print("DBG: self.vehicleRightDistance = {0}".format(self.vehicleRightDistance))
      #print("DBG: Constants.DISTANCE_NOISE = {0}".format(Constants.DISTANCE_NOISE))
      rightDistPDF = norm.pdf(particleDistRight, self.vehicleRightDistance, Constants.DISTANCE_NOISE)
      # Adding a fraction times the peak probability
      # Chances that the measurment is bad
      self.particles[i][Constants.WEIGHT] *= rightDistPDF + 0.003 * 1.33

      #print("DBG: headingPDF = {0}".format(headingPDF))
      #print("DBG: leftDistPDF = {0}".format(leftDistPDF))
      #print("DBG: rightDistPDF = {0}\n".format(rightDistPDF))

  #-------------------------------------------------------------------------------
  def _generateNewParticleList(self):
    '''
    Calculate the cumulative sum of the particle weights, then generates new particles using the cumulative sum over a random distribution
    '''
    self.cumulativeSum = [ sum([ p[Constants.WEIGHT] for p in self.particles[ : i + 1] ]) for i in range(len(self.particles)) ]
    genParticles = []
    for i in range(len(self.particles)):
      genCumSum = random.uniform(0.0, self.cumulativeSum[-1])
      # Get the index of the particle we want to generate
      particleIndex = 0
      for csi in range(len(self.cumulativeSum)):
        if self.cumulativeSum[csi] >= genCumSum:
          particleIndex = csi
          break
      genParticles.append(particleIndex)

    # Generate particles
    self.particles = [ [ self.particles[genP][Constants.X], self.particles[genP][Constants.Y], self.particles[genP][Constants.HEADING], 1.0 ] for genP in genParticles ]

  #-------------------------------------------------------------------------------
  def _calculateDistanceLineOfSight(self, particle):
    '''
    Calculate the distance line of sight and intersection of the given particle
    @param particle - list describing the particle distance line of sight [X, Y, Heading, Weight]
    @return particle closest distance intersection on its left side
    @return particle closest distance intersection on its right side
    '''
    particleDistLeft, particleDistRight = 0.0, 0.0

    # Left Distance
    rX, rY = rotate(Constants.DIST_LEFT_SENSOR_POSITION[Constants.X], Constants.DIST_LEFT_SENSOR_POSITION[Constants.Y], particle[Constants.HEADING])
    #print("DBG: Rotate Left (rX, rY) = ({0}, {1})".format(rX, rY))
    leftStartPoint = [ particle[Constants.X] + rX, particle[Constants.Y] + rY ]
    #print("DBG: leftStartPoint(X, Y) = {0}".format(leftStartPoint))

    rX, rY = rotate(Constants.DIST_MAX_DISTANCE, 0.0, particle[Constants.HEADING] + Constants.DIST_LEFT_SENSOR_ORIENTATION )
    #print("DBG: Rotate Left (rX, rY) = ({0}, {1})".format(rX, rY))
    leftEndPoint = [ leftStartPoint[Constants.X] + rX,  leftStartPoint[Constants.Y] + rY ]
    #print("DBG: leftEndPoint(X, Y) = {0}".format(leftEndPoint))
    leftDistLine = Line(leftStartPoint, leftEndPoint)

    # Right Distance
    rX, rY = rotate(Constants.DIST_RIGHT_SENSOR_POSITION[Constants.X], Constants.DIST_RIGHT_SENSOR_POSITION[Constants.Y], particle[Constants.HEADING])
    #print("DBG: Rotate Right (rX, rY) = ({0}, {1})".format(rX, rY))
    rightStartPoint = [ particle[Constants.X] + rX, particle[Constants.Y] + rY ]
    #print("DBG: rightStartPoint(X, Y) = {0}".format(rightStartPoint))

    rX, rY = rotate(Constants.DIST_MAX_DISTANCE, 0.0, particle[Constants.HEADING] + Constants.DIST_RIGHT_SENSOR_ORIENTATION )
    #print("DBG: Rotate Right (rX, rY) = ({0}, {1})".format(rX, rY))
    rightEndPoint = [ rightStartPoint[Constants.X] + rX,  rightStartPoint[Constants.Y] + rY ]
    #print("DBG: rightEndPoint(X, Y) = {0}".format(rightEndPoint))
    rightDistLine = Line(rightStartPoint, rightEndPoint)

    # Get left intersections with map walls
    leftIntersections = []
    for c in self.courseMap.circles:
      inter = c.findIntersection(leftDistLine)
      if inter is not None:
        for i in inter:
          if i is not None:
            leftIntersections += [ i ]

    leftIntersections += [ l.findIntersection(leftDistLine) for l in self.courseMap.lines ]

    #print("DBG: leftIntersections = {0}".format(leftIntersections))

    # Get right intersections with map walls
    rightIntersections = []
    for c in self.courseMap.circles:
      inter = c.findIntersection(leftDistLine)
      if inter is not None:
        for i in inter:
          if i is not None:
            rightIntersections += [ i ]

    rightIntersections += [ l.findIntersection(rightDistLine) for l in self.courseMap.lines ]
    #print("DBG: rightIntersections = {0}".format(rightIntersections))

    # Calculate distances
    try:
      particleDistLeft = math.sqrt(min([ math.pow(li[Constants.X] - leftStartPoint[Constants.X], 2.0) + math.pow(li[Constants.Y] - leftStartPoint[Constants.Y], 2.0) for li in leftIntersections if li is not None]))
    except:
      particleDistLeft = None

    try:
      particleDistRight = math.sqrt(min([ math.pow(ri[Constants.X] - rightStartPoint[Constants.X], 2.0) + math.pow(ri[Constants.Y] - rightStartPoint[Constants.Y], 2.0) for ri in rightIntersections if ri is not None ]))
    except:
      particleDistRight = None

    # Sanity check
    if particleDistLeft is None or particleDistLeft >= Constants.DIST_MAX_DISTANCE:
      particleDistLeft = Constants.DIST_MAX_DISTANCE + 2.0 * Constants.DISTANCE_NOISE

    if particleDistRight is None or particleDistRight >= Constants.DIST_MAX_DISTANCE:
      particleDistRight = Constants.DIST_MAX_DISTANCE + 2.0 * Constants.DISTANCE_NOISE

    return particleDistLeft, particleDistRight

  #-------------------------------------------------------------------------------
  def _debugDescription(self):
    '''
    Generates debugging information about the particle filter
    @return string describing debug information
    '''
    desc = "Particle Filter:\n"
    desc += "\tparticleNumber = {0}\n".format(len(self.particles))
    desc += "\tprevTime = {0}\n".format(self.prevTime)
    desc += "\tcurrentTime = {0}\n".format(self.currentTime)
    desc += "\tdt = {0}\n".format(self.dt)
    desc += "\tvehicleVelocity = {0}\n".format(self.vehicleVelocity)
    desc += "\tvehicleHeading = {0}\n".format(self.vehicleHeading)
    desc += "\tvehicleLeftDistance = {0}\n".format(self.vehicleLeftDistance)
    desc += "\tvehicleRightDistance = {0}\n".format(self.vehicleRightDistance)
    desc += "\tvehicleSteeringAngle = {0}\n".format(self.vehicleSteeringAngle)
    desc += "\tcumulativeSum = {0}\n".format(self.cumulativeSum)
    desc += "\tparticles:\n"
    for p in sorted(self.particles, key=lambda x: x[Constants.WEIGHT], reverse=True):
      desc += "\t\t[X = {0:.4f}, Y = {1:.4f}, H = {2:.4f}, W = {3:.4f}]\n".format(p[0], p[1], math.degrees(p[2]), p[3])

    desc += "\testVehicleX = {0}\n".format(self.estVehicleX)
    desc += "\testVehicleY = {0}\n".format(self.estVehicleY)
    desc += "\testVehicleHeading = {0}\n".format(self.estVehicleHeading)
    desc += "\tcovarVehicle:\n"
    for c in self.covarVehicle:
      desc += "\t\t{0}\n".format(c)
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

