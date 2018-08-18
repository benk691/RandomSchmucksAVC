import time

class PID:
  '''
  General PID control class
  '''
  #-------------------------------------------------------------------------------
  def __init__(self, P, I, D, windup):
    '''
    Initializes the PID control
    @param P - proportional constant
    @param I - integral constant
    @param D - derivative constant
    @param windup - windup constant
    '''
    self.P = P
    self.I = I
    self.D = D
    self.windup = windup
    self.currentTime = time.time()
    self.accumulator = 0.0
    self.error = 0.0
    self.prevError = 0.0
    self.prevTime = 0.0
    self.measurement = 0.0
    self.goal = 0.0
    self.dt = 0.0
    self.controlSignal = 0.0
    self.sign = lambda x: x and (1.0, -1.0)[x < 0.0]
  
  #-------------------------------------------------------------------------------
  def setGoal(self, goal):
    '''
    Set the goal of the PID control
    @param goal - the goal of the PID control
    '''
    self.goal = goal

  #-------------------------------------------------------------------------------
  def setMeasurement(self, measurement):
    '''
    Set the current measurement of the sensor we are trying to control
    @param measurement - the current measurement
    '''
    self.measurement = measurement
    self.currentTime = time.time()

  #-------------------------------------------------------------------------------
  def control(self):
    '''
    Calculates the control signal to control the sensor towards its goal
    @return the control signal
    '''
    self._calculateError()
    if self.dt == 0.0:
      self.dt = 0.1
    self.controlSignal = self.P * self.error + self.D * (self.error - self.prevError) / self.dt + self.I * self.accumulator
    self.prevError = self.error
    return self.controlSignal

  #-------------------------------------------------------------------------------
  def _calculateError(self):
    '''
    Use the current measurement to calculate the error from the desired goal
    '''
    self.error = self.goal - self.measurement
    # delta time
    if self.prevTime != 0.0:
      self.dt = self.currentTime - self.prevTime
    else:
      self.dt = 0.1
    self.prevTime = self.currentTime
    self.accumulator += self.error * self.dt
    if abs(self.accumulator) > self.windup:
      self.accumulator = self.windup * self.sign(self.accumulator)
  
  #-------------------------------------------------------------------------------
  def _debugDescription(self):
    '''
    Generates debugging information about the PID control
    @return string describing debug information
    '''
    desc = "PID:\n"
    desc += "\tP = {0}\n".format(self.P)
    desc += "\tI = {0}\n".format(self.I)
    desc += "\tD = {0}\n".format(self.D)
    desc += "\twindup = {0}\n".format(self.windup)
    desc += "\taccumulator = {0}\n".format(self.accumulator)
    desc += "\tprevError = {0}\n".format(self.prevError)
    desc += "\terror = {0}\n".format(self.error)
    desc += "\tprevTime = {0}\n".format(self.prevTime)
    desc += "\tcurrentTime = {0}\n".format(self.currentTime)
    desc += "\tdt = {0}\n".format(self.dt)
    desc += "\tmeasurement = {0}\n".format(self.measurement)
    desc += "\tgoal = {0}\n".format(self.goal)
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

