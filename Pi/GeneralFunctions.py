import math
import Constants

'''
General functions that are used across the code
'''

#-------------------------------------------------------------------------------
def staticVars(**kwargs):
  '''
  Decorator that creates the static variables listed
  Credit for code: ony on Stack Overflow
  @param kwargs - the vstatic variables and their values
  @return the decorated function with its static variables
  '''
  def decorate(func):
    '''
    Decorates a function
    @param func - the function to decorate
    @return the decorated function
    '''
    for k in kwargs:
      setattr(func, k ,kwargs[k])
    return func
  return decorate

#-------------------------------------------------------------------------------
'''
Converts seconds to milli-seconds
@param sec - number of seconds
'''
convertSecToMilliSec = lambda sec : sec * Constants.MILLI_SEC_IN_SEC

#-------------------------------------------------------------------------------
'''
Converts millis-seconds to seconds
@param ms - number of milli-seconds
'''
convertMilliSecToSec = lambda ms : ms / Constants.MILLI_SEC_IN_SEC

#-------------------------------------------------------------------------------
'''
Converts feet to meters
@param ft - the number of feet
@return the number of meters
'''
convertFtToM = lambda ft: ft * Constants.FEET_IN_METER

#-------------------------------------------------------------------------------
def wrapAngle(angle):
  '''
  Wraps the angle [0,360] [degrees]
  @param angle - the angle to wrap [degrees]
  @return the wrapped angle [degrees]
  '''
  while angle > 180.0:
    angle -= 360.0

  while angle < -180.0:
    angle += 360.0

  return angle

#-------------------------------------------------------------------------------
def unwrapAngle(angle, prevAngle):
  '''
  Unwraps the angle [0,360] [degrees]
  @param angle - the angle to unwrap [degrees]
  @param prevAngle - the previous angle read in [degrees]
  @return the unwrapped angle [degrees]
  '''
  angleShift = wrapAngle(angle - prevAngle)
  angle = angleShift + prevAngle
  prevAngle = angle
  return angle, prevAngle
  
#-------------------------------------------------------------------------------
def rotate(x, y, theta):
  '''
  Rotates the X, Y coordinate by theta
  x - the X coordinate
  y - the Y coordinate
  theta - the amount to rotate (radians)
  '''
  outX = x * math.cos(theta) - y * math.sin(theta)
  outY = x * math.sin(theta) + y * math.cos(theta)
  return outX, outY

