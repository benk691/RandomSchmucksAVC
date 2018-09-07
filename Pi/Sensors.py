import time
import random
import Constants
from gpiozero import DistanceSensor
import Adafruit_ADS1x15
from Adafruit_BNO055 import BNO055

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
    self.adc = Adafruit_ADS1x15.ADS1115()
    self.imu = BNO055.BNO055()
    if not self.imu.begin():
      raise RuntimeError('Failed to initialize IMU! Is the sensor connected?')
    # Distance sensors
    self.leftDistSensor = DistanceSensor(echo=Constants.DIST_LEFT_ECHO_PIN, trigger=Constants.DIST_LEFT_TRIGGER_PIN, max_distance=Constants.DIST_MAX_DISTANCE, queue_len=Constants.DIST_QUEUE_LENGTH)
    self.rightDistSensor = DistanceSensor(echo=Constants.DIST_RIGHT_ECHO_PIN, trigger=Constants.DIST_RIGHT_TRIGGER_PIN, max_distance=Constants.DIST_MAX_DISTANCE, queue_len=Constants.DIST_QUEUE_LENGTH)
    # Sensor values (-0 means that the sensor is not reading data)
    self.steeringPotValue = -0
    self.leftTachValue = -0
    self.rightTachValue = -0
    self.leftDistance = -0
    self.rightDistance = -0
    self.heading = -0
    self.roll = -0
    self.pitch = -0
    self.sysCal = -0
    self.gyroCal = -0
    self.accelCal = -0
    self.magCal = -0

  #-------------------------------------------------------------------------------
  def read(self):
    '''
    Reads all sensor values
    '''
    #st = time.time()
    self.steeringPotValue = self.adc.read_adc(Constants.ADC_POT_CHNL, gain=Constants.ADC_GAIN, data_rate=Constants.ADC_DATA_RATE) 
    #print("steeringPotValue = {0}".format(self.steeringPotValue))
    #print("DBG: Pot = {0}".format(time.time() - st))
    #st = time.time()
    self.rightTachValue = self.adc.read_adc(Constants.ADC_RIGHT_WHEEL_CHNL, gain=Constants.ADC_GAIN, data_rate=Constants.ADC_DATA_RATE)

    #print("rightTachValue = {0}".format(self.rightTachValue))
    #print("DBG: rightTach = {0}".format(time.time() - st))
    #st = time.time()
    self.leftTachValue = self.adc.read_adc(Constants.ADC_LEFT_WHEEL_CHNL, gain=Constants.ADC_GAIN, data_rate=Constants.ADC_DATA_RATE)

    #print("leftTachValue = {0}".format(self.leftTachValue))
    #print("DBG: leftTach = {0}".format(time.time() - st))
    #st = time.time()
    #self.rightDistance = self.rightDistSensor.distance
    #print("rightDistance = {0}".format(self.rightDistance))
    #print("DBG: rDist = {0}".format(time.time() - st))
    #st = time.time()
    #self.leftDistance = self.leftDistSensor.distance
    #print("leftDistance = {0}".format(self.leftDistance))
    #print("DBG: lDist = {0}".format(time.time() - st))
    #st = time.time()
    # TODO: third front distance sensor?
    # NOTE: the read in angles are in degrees
    self.heading, self.roll, self.pitch = self.imu.read_euler()
    self.sysCal, self.gyroCal, self.accelCal, self.magCal = self.imu.get_calibration_status()

    #print("read imu")
    #print("DBG: imu = {0}".format(time.time() - st))

  #-------------------------------------------------------------------------------
  def _debugDescription(self):
    '''
    Generates debugging information about the sensors
    @return string describing debug information
    '''
    desc = "Sensor Info:\n"
    desc += "\tSteering Pot = {0}\n".format(self.steeringPotValue)
    desc += "\tRight Tach = {0}\n".format(self.rightTachValue)
    desc += "\tLeft Tach = {0}\n".format(self.leftTachValue)
    desc += "\tRight Distance = {0}\n".format(self.rightDistance)
    desc += "\tLeft Distance = {0}\n".format(self.leftDistance)
    desc += "\tHeading = {0}\n".format(self.heading)
    desc += "\tRoll = {0}\n".format(self.roll)
    desc += "\tPitch = {0}\n".format(self.pitch)
    desc += "\tSys Cal = {0}\n".format(self.sysCal)
    desc += "\tGyro Cal = {0}\n".format(self.gyroCal)
    desc += "\tAccel Cal = {0}\n".format(self.accelCal)
    desc += "\tMag Cal = {0}\n".format(self.magCal)
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

