#!/usr/bin/python3.5
import math
import Constants
from Map import Map
from CourseMap import CourseMap
from TestMap import TestMap
from ParticleFilter import ParticleFilter

fi = 0
filename = 'test-{0}.svg'.format(fi)

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
m = TestMap()
N = 300
startBox = [[0.0, 0.0], [1.0, 1.0]]
headingRange = [5.0, 10.0]
pf = ParticleFilter(N, startBox, headingRange, m)

Constants.VEHICLE_AXLE_LEN = 1.0
Constants.STEERING_ANGLE_NOISE = math.radians(10.0)
Constants.VELOCITY_NOISE = 0.01
Constants.HEADING_NOISE = math.radians(5.0)
Constants.DISTANCE_NOISE = 0.3

pf.vehicleVelocity = 2.0
pf.vehicleSteeringAngle = math.radians(0.0)
pf.vehicleHeading = math.radians(0.0)
pf.vehicleLeftDistance = Constants.DIST_MAX_DISTANCE + 0.6
pf.vehicleRightDistance = Constants.DIST_MAX_DISTANCE + 0.6
pf.dt = 1.0

pf.particles = [ [0.0, -20.0, 0.0, 1.0, 0.0, 0.0] for p in range(N) ]
print(pf)

for i in range(10):
  pf._scatterPlotParticles(filename)
  pf._predict()
  print(pf)
  pf._scatterPlotParticles(updateFName())

  pf._weight()
  pf._scatterPlotParticles(updateFName())
  print(pf)

  pf._generateNewParticleList()
  pf._scatterPlotParticles(updateFName())
  print(pf)

