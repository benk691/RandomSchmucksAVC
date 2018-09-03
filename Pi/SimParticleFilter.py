import random
import math
import time
import numpy
import matplotlib.pyplot as matplot
import matplotlib.lines as mlines
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection
import Constants
from matplotlib.colors import Normalize
from scipy.stats import norm
from Line import Line

class SimParticleFilter:
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
  def getEstiamtedVehicleLocation(self):
    '''
    Gets the estimated vehicle location based on the particles
    @return mean(X), mean(Y), mean(heading), covariance
    '''
    print("DBG: getEstiamtedVehicleLocation called")
    self.currentTime = time.time()
    #self.dt = self.currentTime - self.prevTime
    self.dt = 0.1
    # Perform particle calculations
    self._predict()
    self._weight()
    self._generateNewParticleList()
    # Calculate estimates
    #print("DBG: GE 1")
    self.estVehicleX = sum([ p[Constants.X] for p in self.particles]) / float(len(self.particles))

    #print("DBG: GE 2")
    self.estVehicleY = sum([ p[Constants.Y] for p in self.particles]) / float(len(self.particles))
    #print("DBG: GE 3")
    self.estVehicleHeading = sum([ p[Constants.HEADING] for p in self.particles]) / float(len(self.particles))
    #print("DBG: GE 4")
    # Calculate covariance
    x = [ p[Constants.X] for p in self.particles ]
    #print("DBG: GE 5")
    y = [ p[Constants.Y] for p in self.particles ]
    #print("DBG: GE 6")
    h = [ p[Constants.HEADING] for p in self.particles ]
    #print("DBG: GE 7")
    self.covarVehicle = numpy.cov(numpy.vstack((x,y,h)))
    #print("DBG: GE 8")
    return self.estVehicleX, self.estVehicleY, self.estVehicleHeading, self.covarVehicle

  #-------------------------------------------------------------------------------
  def _getVehicleMeasurements(self):
    '''
    Read the sensor measurements from the vehicle
    '''
    # TODO: Connect this with the sensor conversion/filtered reading thread
    self.vehicleVelocity = 0.0
    self.vehicleHeading = 0.0
    self.vehicleLeftDistance = 0.0
    self.vehicleRightDistance = 0.0
    self.vehicleSteeringAngle = 0.0
    # TODO: Put this in the sensor conversion thread
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
    print("DBG: _predict called")
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
      # Generate a velocity for the particles
      genVelocity = random.gauss(mu=self.vehicleVelocity, sigma=Constants.VELOCITY_NOISE) * self.dt
      rotatedX, rotatedY = self._rotate(turnRadius * math.sin(genVelocity / turnRadius), turnRadius * (1 - math.cos(genVelocity / turnRadius)), self.particles[i][Constants.HEADING])
      print("DBG: genSteeringAngle = {0}".format(genSteeringAngle))
      print("DBG: turnRadius = {0}".format(turnRadius))
      print("DBG: genVelocity = {0}".format(genVelocity))
      print("DBG: rotatedX, rotatedY = ({0}, {1})\n".format(rotatedX, rotatedY))
      self.particles[i][Constants.X] += rotatedX
      self.particles[i][Constants.Y] += rotatedY
      self.particles[i][Constants.HEADING] += genVelocity / turnRadius

  #-------------------------------------------------------------------------------
  def _weight(self):
    '''
    Calculates the weights for the given particle
    '''
    print("DBG: _weight called")
    for i in range(len(self.particles)):
      print("DBG: vehicleHeading = {0}".format(self.vehicleHeading))
      particleDistLeft, particleDistRight = self._calculateDistanceLineOfSight(self.particles[i])
      print("DBG: particleDistLeft = {0}".format(particleDistLeft))
      print("DBG: particleDistRight = {0}".format(particleDistRight))
      # PDF(measurement, mean, std_dev)
      print("DBG: self.particles[{0}][Constants.HEADING] = {1}".format(i, self.particles[i][Constants.HEADING]))
      print("DBG: self.vehicleHeading = {0}".format(self.vehicleHeading))
      print("DBG: Constants.HEADING_NOISE = {0}".format(Constants.HEADING_NOISE))
      headingPDF = norm.pdf(self.particles[i][Constants.HEADING], self.vehicleHeading, Constants.HEADING_NOISE)
      # Adding a fraction times the peak probability
      # Chances that the measurment is bad
      self.particles[i][Constants.WEIGHT] *= headingPDF + 0.03 * 7.62

      print("DBG: particleDistLeft = {0}".format(particleDistLeft))
      print("DBG: self.vehicleLeftDistance = {0}".format(self.vehicleLeftDistance))
      print("DBG: Constants.DISTANCE_NOISE = {0}".format(Constants.DISTANCE_NOISE))
      leftDistPDF = norm.pdf(particleDistLeft, self.vehicleLeftDistance, Constants.DISTANCE_NOISE)
      # Adding a fraction times the peak probability
      # Chances that the measurment is bad
      self.particles[i][Constants.WEIGHT] *= leftDistPDF + 0.003 * 1.33

      print("DBG: particleDistRight = {0}".format(particleDistRight))
      print("DBG: self.vehicleRightDistance = {0}".format(self.vehicleRightDistance))
      print("DBG: Constants.DISTANCE_NOISE = {0}".format(Constants.DISTANCE_NOISE))
      rightDistPDF = norm.pdf(particleDistRight, self.vehicleRightDistance, Constants.DISTANCE_NOISE)
      # Adding a fraction times the peak probability
      # Chances that the measurment is bad
      self.particles[i][Constants.WEIGHT] *= rightDistPDF + 0.003 * 1.33

      print("DBG: headingPDF = {0}".format(headingPDF))
      print("DBG: leftDistPDF = {0}".format(leftDistPDF))
      print("DBG: rightDistPDF = {0}\n".format(rightDistPDF))

  #-------------------------------------------------------------------------------
  def _generateNewParticleList(self):
    '''
    Calculate the cumulative sum of the particle weights, then generates new particles using the cumulative sum over a random distribution
    '''
    print("DBG: _generateNewParticleList called")
    self.cumulativeSum = [ sum([ p[Constants.WEIGHT] for p in self.particles[ : i + 1] ]) for i in range(len(self.particles)) ]
    #print("DBG: 1")
    genParticles = []
    for i in range(len(self.particles)):
      genCumSum = random.uniform(0.0, self.cumulativeSum[-1])
      #print("DBG: 2")
      # Get the index of the particle we want to generate
      particleIndex = 0
      for csi in range(len(self.cumulativeSum)):
        if self.cumulativeSum[csi] >= genCumSum:
          particleIndex = csi
          break
      genParticles.append(particleIndex)

    #print("DBG: 3")
    # Generate particles
    self.particles = [ [ self.particles[genP][Constants.X], self.particles[genP][Constants.Y], self.particles[genP][Constants.HEADING], 1.0 ] for genP in genParticles ]
    #print("DBG: 4")

  #-------------------------------------------------------------------------------
  def _calculateDistanceLineOfSight(self, particle):
    '''
    Calculate the distance line of sight and intersection of the given particle
    @param particle - list describing the particle distance line of sight [X, Y, Heading, Weight]
    @return particle closest distance intersection on its left side
    @return particle closest distance intersection on its right side
    '''
    print("DBG: _calculateDistanceLineOfSight called")
    particleDistLeft, particleDistRight = 0.0, 0.0

    # Left Distance
    rX, rY = self._rotate(Constants.DIST_LEFT_SENSOR_POSITION[Constants.X], Constants.DIST_LEFT_SENSOR_POSITION[Constants.Y], particle[Constants.HEADING])
    print("DBG: Rotate Left (rX, rY) = ({0}, {1})".format(rX, rY))
    leftStartPoint = [ particle[Constants.X] + rX, particle[Constants.Y] + rY ]
    print("DBG: leftStartPoint(X, Y) = {0}".format(leftStartPoint))

    rX, rY = self._rotate(Constants.DIST_MAX_DISTANCE, 0.0, particle[Constants.HEADING] + Constants.DIST_LEFT_SENSOR_OREINTATION )
    print("DBG: Rotate Left (rX, rY) = ({0}, {1})".format(rX, rY))
    leftEndPoint = [ leftStartPoint[Constants.X] + rX,  leftStartPoint[Constants.Y] + rY ]
    print("DBG: leftEndPoint(X, Y) = {0}".format(leftEndPoint))
    leftDistLine = Line(leftStartPoint, leftEndPoint)

    # Right Distance
    rX, rY = self._rotate(Constants.DIST_RIGHT_SENSOR_POSITION[Constants.X], Constants.DIST_RIGHT_SENSOR_POSITION[Constants.Y], particle[Constants.HEADING])
    print("DBG: Rotate Right (rX, rY) = ({0}, {1})".format(rX, rY))
    rightStartPoint = [ particle[Constants.X] + rX, particle[Constants.Y] + rY ]
    print("DBG: rightStartPoint(X, Y) = {0}".format(rightStartPoint))

    rX, rY = self._rotate(Constants.DIST_MAX_DISTANCE, 0.0, particle[Constants.HEADING] + Constants.DIST_RIGHT_SENSOR_OREINTATION )
    print("DBG: Rotate Right (rX, rY) = ({0}, {1})".format(rX, rY))
    rightEndPoint = [ rightStartPoint[Constants.X] + rX,  rightStartPoint[Constants.Y] + rY ]
    print("DBG: rightEndPoint(X, Y) = {0}".format(rightEndPoint))
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

    print("DBG: leftIntersections = {0}".format(leftIntersections))

    # Get right intersections with map walls
    rightIntersections = []
    for c in self.courseMap.circles:
      inter = c.findIntersection(leftDistLine)
      if inter is not None:
        for i in inter:
          if i is not None:
            rightIntersections += [ i ]

    rightIntersections += [ l.findIntersection(rightDistLine) for l in self.courseMap.lines ]
    print("DBG: rightIntersections = {0}".format(rightIntersections))

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
  def _rotate(self, x, y, theta):
    '''
    Rotates the X, Y coordinate by theta
    x - the X coordinate
    y - the Y coordinate
    theta - the amount to rotate (radians)
    '''
    print("DBG: _rotate called")
    print("DBG: x = {0}, y = {1}, theta = {2}".format(x, y, theta))
    outX = x * math.cos(theta) - y * math.sin(theta)
    outY = x * math.sin(theta) + y * math.cos(theta)
    return outX, outY

  #-------------------------------------------------------------------------------
  def _scatterPlotParticles(self, car, filename):
    '''
    Create a scatter plot of the particle positions, headings and weight
    @param car - Car class
    @param filename - the SVG filename to write the scatterplot to
    '''
    #matplot.xlim(-0.5,25.5)
    #matplot.ylim(-25.0,-15.0)
    # Plot Particles
    self._drawCourseMap()
    x = [ p[Constants.X] for p in self.particles ]
    y = [ p[Constants.Y] for p in self.particles ]
    h = [ p[Constants.HEADING] for p in self.particles ]
    totalWeight = sum([ p[Constants.WEIGHT] for p in self.particles ])
    w = [ p[Constants.WEIGHT] / totalWeight for p in self.particles ]

    matplot.scatter(x, y, c=w)

    self._drawEstimate()

    self._drawCar(car)

    matplot.savefig(filename)
    matplot.clf()

    # STREAMPLOT ATTEMPT
    #u = []
    #v = []
    #for inX, inY, inH in zip(x, y, h):
    #  outU, outV = self._rotate(inX, inY, inH)
    #  u.append(outU)
    #  v.append(outV)
    #
    #u, v = numpy.mgrid[x:y:1j, u:v:1j]
    ##matplot.streamplot(x, y, [ [ i for i in u ] for i in u ], [ [ i for i in v ] for i in v ], color=w)
    #matplot.streamplot(numpy.array(x), numpy.array(y), u, v, color=w)

    #cRX, cRY = self._rotate(car.x, car.y, car.heading)
    #cU, cV = numpy.mgrid[car.x:car.y:1j, cRX:cRY:1j] 
    #matplot.streamplot(numpy.array([car.x]), numpy.array([car.y]), cU, cV, marker='+', color='pink')

    # ALPHAS
    #matplot.scatter(x, y, c=w)
    #maxWeight = max([ p[Constants.WEIGHT] for p in self.particles ])
    #w = [ p[Constants.WEIGHT] / maxWeight for p in self.particles ]
    #
    ## RGBA
    #colors = [ [ g, 0, 1-g, 0.2] for g in w ]
    #matplot.scatter(x, y, c=colors)
    #matplot.savefig(filename)

  #-------------------------------------------------------------------------------
  def _drawCar(self, car):
    '''
    Draws the vehicle on the map
    '''
    # Get left distance sensor position
    rX, rY = self._rotate(Constants.DIST_LEFT_SENSOR_POSITION[Constants.X], Constants.DIST_LEFT_SENSOR_POSITION[Constants.Y], car.heading)
   
    leftStartPoint = [ car.x + rX, car.y + rY ]
    
    # Get right distance sensor position
    rX, rY = self._rotate(Constants.DIST_RIGHT_SENSOR_POSITION[Constants.X], Constants.DIST_RIGHT_SENSOR_POSITION[Constants.Y], car.heading)
  
    rightStartPoint = [ car.x + rX, car.y + rY ]

    # Plot distance sensors
    matplot.scatter(leftStartPoint[Constants.X], rightStartPoint[Constants.Y], marker='<', color='green')

    matplot.scatter(rightStartPoint[Constants.X], rightStartPoint[Constants.Y], marker='>', color='green')
    
    # Plot car
    matplot.scatter([car.x], [car.y], marker='+', color='green')

  #-------------------------------------------------------------------------------
  def _drawEstimate(self):
    '''
    Draws the estimated vehicle on the map
    '''
    # Get left distance sensor position
    rX, rY = self._rotate(Constants.DIST_LEFT_SENSOR_POSITION[Constants.X], Constants.DIST_LEFT_SENSOR_POSITION[Constants.Y], self.estVehicleHeading)
   
    leftStartPoint = [ self.estVehicleX + rX, self.estVehicleY + rY ]
    
    # Get right distance sensor position
    rX, rY = self._rotate(Constants.DIST_RIGHT_SENSOR_POSITION[Constants.X], Constants.DIST_RIGHT_SENSOR_POSITION[Constants.Y], self.estVehicleHeading)
  
    rightStartPoint = [ self.estVehicleX + rX, self.estVehicleY + rY ]

    # Plot distance sensors
    matplot.scatter(leftStartPoint[Constants.X], rightStartPoint[Constants.Y], marker='<', color='orange')

    matplot.scatter(rightStartPoint[Constants.X], rightStartPoint[Constants.Y], marker='>', color='orange')
    
    # Plot car
    matplot.scatter([self.estVehicleX], [self.estVehicleY], marker='+', color='orange')

  #-------------------------------------------------------------------------------
  def _drawCourseMap(self):
    '''
    Draws the course map
    '''
    # Draw lines
    for l in self.courseMap.lines:
      self._drawLine(l)

    # Draw circles
    for c in self.courseMap.circles:
      self._drawCircle(c)

    self._drawWaypoints()

  #-------------------------------------------------------------------------------
  def _drawLine(self, line):
    '''
    Draw a line
    @param Line class to draw
    '''
    ax = matplot.gca()
    l = mlines.Line2D([line.startPoint[Constants.X], line.endPoint[Constants.X]], [line.startPoint[Constants.Y], line.endPoint[Constants.Y]]) 
    ax.add_line(l)

  #-------------------------------------------------------------------------------
  def _drawCircle(self, circle):
    '''
    Draw a cirlce
    @param circle - Circle class to draw
    '''
    ax = matplot.gca()
    c = matplot.Circle(circle.center, circle.radius, fill=False) 
    ax.add_patch(c)

  #-------------------------------------------------------------------------------
  def _drawWaypoints(self):
    '''
    Draws a waypoints
    '''
    x = [ w[Constants.X] for w in self.courseMap.waypoints ]
    y = [ w[Constants.Y] for w in self.courseMap.waypoints ]
    matplot.scatter(x, y, marker='^', color='red')

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

