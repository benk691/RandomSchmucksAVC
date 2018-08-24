#!/usr/bin/python3.5
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
N = 10
startBox = [[0.0, 0.0], [1.0, 1.0]]
headingRange = [5.0, 10.0]
pf = ParticleFilter(N, startBox, headingRange, m)

print(pf)
pf._scatterPlotParticles(filename)
pf._generateNewParticleList()
print(pf)
pf._scatterPlotParticles(updateFName())
