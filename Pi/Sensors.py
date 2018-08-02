from Constants import Constants
from gpiozero import DistanceSensor

class Sensors:
  '''
  Keeps track of all the sensor data
  '''
  #-------------------------------------------------------------------------------
  def __init__(self):
    '''
    Initializes all the sensors we want to consume data from
    '''
    # I2C bus readers
    self.pwm = Adafruit_PCA9685.PCA9685()
    self.adc = Adafruit_ADS1x15.ADS1115()
    # Distance sensors
    self.leftDistSensor = DistanceSensor(echo=Constants.DIST_LEFT_ECHO_PIN, trigger=Constants.DIST_LEFT_TRIGGER_PIN, max_distance=Constants.DIST_QUEUE_LENGTH)
    self.rightDistSensor = DistanceSensor(echo=Constants.DIST_RIGHT_ECHO_PIN, trigger=Constants.DIST_RIGHT_TRIGGER_PIN, max_distance=Constants.DIST_QUEUE_LENGTH)
    # Sensor values (-0 means that the sensor is not reading data)
    self.steeringPotValue = -0
    self.leftTachValue = -0
    self.rightTachValue = -0
    self.leftDistance = -0
    self.rightDistance = -0

  #-------------------------------------------------------------------------------
  def read(self):
    '''
    Reads all sensor values
    '''
    self.steeringPotValue = self.adc.read_adc(Constants.ADC_POT_CHNL, gain=Constants.GAIN, data_rate=Constants.DATA_RATE) 
    self.rightTachValue = self.adc.read_adc(Constants.ADC_RIGHT_WHEEL_CHNL, gain=Constants.GAIN, data_rate=Constants.DATA_RATE)
    self.leftTachValue = self.adc.read_adc(Constants.ADC_LEFT_WHEEL_CHNL, gain=Constants.GAIN, data_rate=Constants.DATA_RATE)
    self.rightDistance = self.rightDistSensor.distance
    self.leftDistance = self.leftDistSensor.distance

  #-------------------------------------------------------------------------------
  def _debugDescription(self):
    desc = "Sensor Info:\n"
    desc += "\tSteering Pot Value = {0}".format(self.steeringPotValue)
    desc += "\tRight Tach Value = {0}".format(self.rightTachValue)
    desc += "\tLeft Tach Value = {0}".format(self.leftTachValue)
    desc += "\tRight Distance Value = {0}".format(self.rightDistance)
    desc += "\tLeft Distance Value = {0}".format(self.leftDistance)
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

