#!/usr/bin/python3.5
import math
import Constants
import random
from Map import Map
from CourseMap import CourseMap
from TestMap import TestMap
from SimParticleFilter import SimParticleFilter
from SimControlPlanner import SimControlPlanner
from scipy.stats import norm

fi = 10

#-------------------------------------------------------------------------------
def updateFName():
  global fi
  global filename
  filename = 'test-{0:03d}.svg'.format(fi)
  print("DBG: filename = {0}".format(filename))
  fi += 1
  return filename

#-------------------------------------------------------------------------------
def moveCar(c, pf):
  genSteeringAngle = pf.vehicleSteeringAngle
  if genSteeringAngle > pf._maxSteeringAngle:
    genSteeringAngle = pf._maxSteeringAngle
  if genSteeringAngle < pf._minSteeringAngle:
    genSteeringAngle = pf._minSteeringAngle
  # TODO: Check solution to division of zero with Brian
  turnRadius = 10000000.0
  if math.tan(genSteeringAngle) != 0.0:
    turnRadius = Constants.VEHICLE_AXLE_LEN / math.tan(genSteeringAngle)
  # Generate a velocity for the particles
  genVelocity = pf.vehicleVelocity * pf.dt
  rotatedX, rotatedY = pf._rotate(turnRadius * math.sin(genVelocity / turnRadius), turnRadius * (1 - math.cos(genVelocity / turnRadius)), c.heading)
  print("DBG: genSteeringAngle = {0}".format(genSteeringAngle))
  print("DBG: turnRadius = {0}".format(turnRadius))
  print("DBG: genVelocity = {0}".format(genVelocity))
  print("DBG: rotatedX, rotatedY = ({0}, {1})\n".format(rotatedX, rotatedY))

  slipX, slipY = pf._rotate(0, random.gauss(0, Constants.SLIP_NOISE * pf.dt), car.heading)
  c.x += rotatedX + slipX
  c.y += rotatedY + slipY
  c.heading += genVelocity / turnRadius

#-------------------------------------------------------------------------------
# Sim only constants
CONTROL_SLIP_NOISE = 0.3
CONTROL_STEERING_ANGLE_NOISE = math.radians(5.0)
CONTROL_VELOCITY_NOISE = 0.3

# car = [x,y,h,steeraAngle, velocity, controlSteeringNoise, controlVelocityNoise]
class Car:
  def __init__(self):
    self.x = 0.0
    self.y = 0.0
    self.heading = 0.0
    self.velocity = 0.0
    self.steeringAngle = 0.0

car = Car()

m = CourseMap()
N = 50
startBox = [[-0.5, -0.5], [0.5, 0.5]]
headingRange = [math.radians(-5.0), math.radians(5.0)]
pf = SimParticleFilter(N, startBox, headingRange, m)

cp = SimControlPlanner(particleFilter=pf, courseMap=m)

Constants.VEHICLE_AXLE_LEN = 0.91
Constants.STEERING_ANGLE_NOISE = math.radians(2.0)
Constants.VELOCITY_NOISE = 0.2
Constants.HEADING_NOISE = math.radians(5.0)
Constants.DISTANCE_NOISE = 0.3
Constants.SLIP_NOISE = 0.4

for i in range(500):
  print('=' * 50)
  print("START LOOP {0}".format(i))
  print('=' * 50)
  
  # Generate measurements
  pf.vehicleVelocity = random.gauss(car.velocity, Constants.VELOCITY_NOISE) + 0.2
  pf.vehicleSteeringAngle = random.gauss(car.steeringAngle, Constants.STEERING_ANGLE_NOISE)
  pf.vehicleHeading = random.gauss(car.heading, Constants.HEADING)
  meanLeftDistance, meanRightDistance = pf._calculateDistanceLineOfSight([car.x, car.y, car.heading, 1.0])
  pf.vehicleLeftDistance = random.gauss(meanLeftDistance, Constants.DISTANCE_NOISE)
  pf.vehicleRightDistance = random.gauss(meanRightDistance, Constants.DISTANCE_NOISE)

  # Run one step
  cp.run()

  print('-' * 50)
  print(cp)
  print('-' * 50)

  car.steeringAngle = random.gauss(cp.steeringAngleGoal, Constants.STEERING_ANGLE_NOISE)
  car.velocity = random.gauss(cp.velocityGoal, Constants.VELOCITY_NOISE)

  if car.steeringAngle > pf._maxSteeringAngle:
    car.steeringAngle = pf._maxSteeringAngle
  if car.steeringAngle < pf._minSteeringAngle:
    car.steeringAngle = pf._minSteeringAngle

  if car.velocity > Constants.MAX_VEHICLE_VELOCITY:
    car.velocity = Constants.MAX_VEHICLE_VELOCITY
  if car.velocity < Constants.MIN_VEHICLE_VELOCITY:
    car.velocity = Constants.MIN_VEHICLE_VELOCITY

  moveCar(car, pf)

  if i % 10 == 0:
    pf._scatterPlotParticles(car, updateFName())

