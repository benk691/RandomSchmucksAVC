#!/usr/bin/python3

DIST_SENSOR = False
MOTOR = False
ARDUINO_PI = True
TENSOR_FLOW = False

#-------------------------------------------------------------------------------
def main():
  while DIST_SENSOR or MOTOR or ARDUINO_PI or TENSOR_FLOW:
    if DIST_SENSOR:
      runDist()
    if MOTOR:
      runMotor()
    if ARDUINO_PI:
      runArduinoPi()
    if TENSOR_FLOW:
      runTensorFlow()

#-------------------------------------------------------------------------------
def runDist():
  '''
  Distance Sensor Play Area
  '''
  from gpiozero import DistanceSensor, MotionSensor
  from time import sleep

  distSensor = DistanceSensor(echo=15, trigger=14, max_distance=2, queue_len=10)

  try:
    while True:
      print('Distance: ', distSensor.distance * 100)
      sleep(0.1)
  except KeyboardInterrupt:
    pass
  finally:
    exit(0)

#-------------------------------------------------------------------------------
def runMotor():
  '''
  Motor Play Area
  '''
  import RPi.GPIO as IO

  motorPin = 19
  dutyCycle = 0
  motorSigFreq = 0

  IO.setwarnings(False)
  IO.setmode(IO.BOARD)
  IO.setup(motorPin, IO.IN)
  motorSignal = IO.PWM(motorPin, motorSigFreq)
  motorSignal.start(dutyCycle)

  try:
    while True:
      pass
  except KeyboardInterupt:
    pass
  finally:
    motorSignal.stop()
    IO.cleanup()
    exit(0)

#-------------------------------------------------------------------------------
def runArduinoPi():
  '''
  Arduino Pi Comm Play area
  '''
  import serial
  import RPi.GPIO
  BAUDRATE = 9600
  USE_I2C = False
  USE_SERIAL = True
  USE_WIFI = False
  if USE_I2C:
    i2cComm()
  elif USE_SERIAL:
    # Setup
    ser = serial.Serial('/dev/ttyACM0', 9600)
    ser.baudrate = BAUDRATE
    serialComm(ser)
  elif USE_WIFI:
    wifiComm()

#-------------------------------------------------------------------------------
def runTensorFlow():
  '''
  Tensor Flow Play Area
  '''
  pass

#-------------------------------------------------------------------------------
def i2cComm():
  '''
  Function to talk to the Arduino over I2C
  '''
  pass


#-------------------------------------------------------------------------------
def serialComm(ser):
  '''
  Function to talk to the Arduino over Serial
  @param ser - the serial communicator
  '''
  from time import sleep  
  readSer = ser.readline()
  print(readSer)
  sleep(1)

#-------------------------------------------------------------------------------
def wifiComm():
  '''
  Function to talk to the Arduino over Wifi
  '''
  pass

#-------------------------------------------------------------------------------
if __name__ == '__main__':
  main()

