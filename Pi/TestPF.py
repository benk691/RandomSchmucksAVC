#!/usr/bin/python3.5
import math
import Constants
from Map import Map
from ParticleFilter import ParticleFilter

fi = 0
filename = 'test-{0}.svg'.format(fi)

#-------------------------------------------------------------------------------
def updateFName():
  global fi
  global filename
  fi += 1
  filename = 'test-{0}.svg'.format(fi)
  return filename

#-------------------------------------------------------------------------------
m = Map()
N = 300
startBox = [[0.0, 0.0], [1.0, 1.0]]
headingRange = [5.0, 10.0]
pf = ParticleFilter(N, startBox, headingRange, m)

Constants.VEHICLE_AXLE_LEN = 1.0
Constants.STEERING_ANGLE_NOISE = math.radians(10.0)
Constants.VELOCITY_NOISE = 0.02
Constants.HEADING_NOISE = math.radians(5.0)
Constants.DISTANCE_NOISE = 0.1

pf.vehicleVelocity = 1.0
pf.vehicleSteeringAngle = 0.0
pf.vehicleHeading = math.radians(30.0)
pf.vehicleLeftDistance = Constants.DIST_MAX_DISTANCE
pf.vehicleRightDistance = Constants.DIST_MAX_DISTANCE
pf.dt = 2.0

pf.particles = [ [0.0, -20.0, 0.0, 1.0] for p in range(N) ]
print(pf)
pf._scatterPlotParticles(filename)
pf._predict()
print(pf)
pf._scatterPlotParticles(updateFName())

pf._weight()
pf._scatterPlotParticles(updateFName())
print(pf)
