#!/usr/bin/python3.5
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
N = 30
startBox = [[0.0, 0.0], [1.0, 1.0]]
headingRange = [5.0, 10.0]
pf = ParticleFilter(N, startBox, headingRange, m)

Constants.VEHICLE_AXLE_LEN = 1.0
Constants.STEERING_ANGLE_NOISE = 5
Constants.VELOCITY_NOISE = 0.1

pf.vehicleVelocity = 1.0
pf.vehicleSteeringAngle = 0.0
pf.dt = 1.0

pf.particles = [ [0.0, 0.0, 0.0, 1.0] for p in range(N) ]
print(pf)
pf._scatterPlotParticles(filename)
pf._predict()
print(pf)
pf._scatterPlotParticles(updateFName())
pf._weight()
pf._scatterPlotParticles(updateFName())
print(pf)
