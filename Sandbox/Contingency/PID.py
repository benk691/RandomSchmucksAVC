import time

class PID:
  #-------------------------------------------------------------------------------
  def __init__(self, pConst, iConst, dConst, windup):
    self.pConst = pConst
    self.iConst = iConst
    self.dConst = dConst
    self.accumulator = 0.0
    self.error = 0.0
    self.prevError = 0.0
    self.windup = windup
    self.currentTime = time.time()
    self.prevTime = 0.0
    self.measurement = 0.0
    self.goal = 0.0
    self.dt = 0.0
  
  #-------------------------------------------------------------------------------
  def setGoal(self, goal):
    self.goal = goal

  #-------------------------------------------------------------------------------
  def setCurrentMeasurement(self, measurement):
    self.measurement = measurement
    self.currentTime = time.time()

  #-------------------------------------------------------------------------------
  def calculateError(self):
    self.error = self.goal - self.measurement
    # delta time
    if self.prevTime != 0.0:
      self.dt = self.currentTime - self.prevTime
    else:
      self.dt = 0.1
    self.prevTime = self.currentTime
    self.accumulator += self.error * self.dt
    if abs(self.accumulator) > self.windup:
      # TODO: get sign of accumulator Do more better
      signOfAccumulator = 1.0
      if self.accumulator < 0.0:
        signOfAccumulator = -1.0
      self.accumulator = self.windup * signOfAccumulator

  #-------------------------------------------------------------------------------
  def control(self):
    self.calculateError()
    if self.dt == 0.0:
      self.dt = 0.1
    self.controlSignal = self.pConst * self.error + self.dConst * (self.error - self.prevError) / self.dt + self.iConst * self.accumulator
    self.prevError = self.error
    return self.controlSignal

  #-------------------------------------------------------------------------------
  def _debugDescription(self):
    desc = "PID:\n"
    desc += "\tP Const = {0}\n".format(self.pConst)
    desc += "\tI Const = {0}\n".format(self.iConst)
    desc += "\tD Const = {0}\n".format(self.dConst)
    desc += "\tAccum = {0}\n".format(self.accumulator)
    desc += "\terror = {0}\n".format(self.error)
    desc += "\tprevError = {0}\n".format(self.prevError)
    desc += "\twindup = {0}\n".format(self.windup)
    desc += "\tprevTime = {0}\n".format(self.prevTime)
    desc += "\tcurrentTime = {0}\n".format(self.currentTime)
    desc += "\tdt = {0}\n".format(self.dt)
    desc += "\tmeasurement = {0}\n".format(self.measurement)
    desc += "\tgoal = {0}\n".format(self.goal)
    return desc

  #-------------------------------------------------------------------------------
  def __repr__(self):
    return self._debugDescription()

  #-------------------------------------------------------------------------------
  def __str__(self):
    return self._debugDescription()

