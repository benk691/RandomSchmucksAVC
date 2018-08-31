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

#-------------------------------------------------------------------------------
def updateFName():
  global fi
  global filename
  fi += 1
  if fi < 10:
    filename = 'test-0{0}.svg'.format(fi)
  else:
    filename = 'test-{0}.svg'.format(fi)
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
  c.x += rotatedX
  c.y += rotatedY
  c.heading += genVelocity / turnRadius

#-------------------------------------------------------------------------------
# Sim only constants
CONTROL_STEERING_ANGLE_NOISE = math.radians(3.0)
CONTROL_VELOCITY_NOISE = 0.2

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
N = 10
startBox = [[0.0, 0.0], [1.0, 1.0]]
headingRange = [5.0, 10.0]
pf = SimParticleFilter(N, startBox, headingRange, m)

cp = SimControlPlanner(particleFilter=pf, courseMap=m)

Constants.VEHICLE_AXLE_LEN = 1.0
Constants.STEERING_ANGLE_NOISE = math.radians(10.0)
Constants.VELOCITY_NOISE = 0.01
Constants.HEADING_NOISE = math.radians(5.0)
Constants.DISTANCE_NOISE = 0.3

for i in range(100):
  # Generate measurements
  pf.vehicleVelocity = random.gauss(car.velocity, Constants.VELOCITY_NOISE)
  pf.vehicleSteeringAngle = random.gauss(car.steeringAngle, Constants.STEERING_ANGLE_NOISE)
  pf.vehicleHeading = random.gauss(car.heading, Constants.HEADING)
  meanLeftDistance, meanRightDistance = pf._calculateDistanceLineOfSight([car.x, car.y, car.heading, 1.0])
  pf.vehicleLeftDistance = random.gauss(meanLeftDistance, Constants.DISTANCE_NOISE)
  pf.vehicleRightDistance = random.gauss(meanRightDistance, Constants.DISTANCE_NOISE)

  # Run one step
  cp.run()

  print(cp)

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
  # TODO: Add graph car is a + and add a direction arrow

  print(cp)


