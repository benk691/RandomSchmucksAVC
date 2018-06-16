
class Vehicle:
  '''
  Controls the Fisher Price Vehicle
  '''
  #-------------------------------------------------------------------------------
  def __init__(self):
    '''
    Initializes the basics of the vehicle
    '''
    speed = 0.0

  #-------------------------------------------------------------------------------
  def setSpeed(self, inSpeed):
    '''
    Sets the speed of the vehicle
    @param inSpeed - the input speed, this should be a decimal value that is used 
                     as an analog input into the Sabertooth H-Bridge
    '''
    speed = inSpeed

  #-------------------------------------------------------------------------------
  def getSpeed(self):
    '''
    Gets the current speed of the vehicle
    @return speed of the vehicle. This is the anlog value being feed into the 
            Sabertooth H-Bridge
    '''
    return speed
  
  #-------------------------------------------------------------------------------
  def drive(self):
    '''
    Drives the vehicle forward
    '''
    pass

  #-------------------------------------------------------------------------------
  def turn(self):
    '''
    Drives the vehicle forward
    '''
    pass

  #-------------------------------------------------------------------------------
  def _debugDescrition():
    '''
    Generates debugging information about the vehicle
    @return string describing debug information
    '''
    desc = "Vehicle Info:\n"
    desc += "\tspeed = {0}\n".format(speed)
    return desc
  
  #-------------------------------------------------------------------------------
  def __repr__(self):
    '''
    Gets the string representation of the class
    @return string representation of the class
    '''
    return _debugDescription()

  #-------------------------------------------------------------------------------
  def __str__(self):
    '''
    Gets the string representation of the class
    @return string representation of the class
    '''
    return _debugDescription()

