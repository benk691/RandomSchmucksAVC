#!/usr/bin/python3.5
import serial
import time
import RPi.GPIO as GPIO
from decimal import Decimal

#-------Constants----------
ARDUINO_SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 9600
HBRIDGE_S1_PIN = 3
HBRIDGE_S2_PIN = 5
DRIVE_PIN = HBRIDGE_S1_PIN
TURN_PIN = HBRIDGE_S2_PIN
HBRIDGE_MOTOR_FREQ = 5000
MAX_RIGHT_TURN_VALUE = 421
CENTER_TURN_VALUE = 483
MAX_LEFT_TURN_VALUE = 605
TURN_TOLERANCE = 10
#--------------------------
DUTY_CYCLE_LEFT = 5.0
DUTY_CYCLE_RIGHT = 2.0

# Turn States
LEFT = 0
RIGHT = 1
STRAIGHT = 2
# Velocity States
FAST = 3
SLOW = 4
# state frequency or how often to chage the state (in secs)
STATE_FREQ = 5
#-----------
STATE = [ STRAIGHT, SLOW ] 
stateCount = 1

# duty cycles
driveDutyCycle = 0.0
turnDutyCycle = 0.0

#-------------------------------------------------------------------------------
def main():
  '''
  Main program
  '''
  try:
    ser, driveMotorSignal, turnMotorSignal = setup()
  
    while True:
      leftVelocity, rightVelocity, potValue = consumeSerialData(ser)
      print('LV: {0}'.format(leftVelocity))
      print('RV: {0}'.format(rightVelocity))
      print('S: {0}'.format(potValue))
      print('driveDutyCycle: {0}'.format(driveDutyCycle))
      print('turnDutyCycle: {0}'.format(turnDutyCycle))
      print('STATE: {0}'.format(STATE))
      controlVelocity(driveMotorSignal, leftVelocity, rightVelocity)
      controlSteering(turnMotorSignal, potValue)
      changeStates()
      time.sleep(1)
  finally:
    driveMotorSignal.stop()
    turnMotorSignal.stop()
    GPIO.cleanup()

#-------------------------------------------------------------------------------
def controlVelocity(driveMotorSignal, leftVelocity, rightVelocity):
  '''
  Controls the vehicles velocity
  @param driveMotorSignal - the signal to control the drive speed
  @param leftVelocity - the velocity of the left wheel
  @param rightVelocity - the velocity of the right wheel
  '''
  global driveDutyCycle

  if STATE[1] == SLOW:
    if (driveDutyCycle - 5.0) <= 0.0 or driveDutyCycle >= 100.0:
      driveDutyCycle = 5.0
    else:
      driveDutyCycle -= 5.0
  elif STATE[1] == FAST:
    if driveDutyCycle <= 0.0 or (driveDutyCycle + 10.0) >= 100.0:
      driveDutyCycle = 50.0
    else:
      driveDutyCycle += 10.0
    
  driveMotorSignal.ChangeDutyCycle(driveDutyCycle)

#-------------------------------------------------------------------------------
def controlSteering(turnMotorSignal, potValue):
  '''
  Controls the steering of the vehicle
  @param turnMotorSignal - the signal that controls the turn speed
  @param potValue - the potentiometer analog value
  '''
  global turnDutyCycle

  if STATE[0] == STRAIGHT:
    if potValue >= (CENTER_TURN_VALUE - TURN_TOLERANCE) and potValue <= (CENTER_TURN_VALUE + TURN_TOLERANCE):
      turnDutyCycle = 0.0
    # To far right
    elif potValue < (CENTER_TURN_VALUE - TURN_TOLERANCE):
      turnDutyCycle = DUTY_CYCLE_LEFT
    # To far left
    else:
      turnDutyCycle = DUTY_CYCLE_RIGHT
  elif STATE[0] == LEFT:
    if potValue >= (MAX_LEFT_TURN_VALUE - TURN_TOLERANCE) and potValue <= (MAX_LEFT_TURN_VALUE + TURN_TOLERANCE):
      turnDutyCycle = 0.0
    # To far right
    elif potValue < (MAX_LEFT_TURN_VALUE - TURN_TOLERANCE):
      turnDutyCycle = DUTY_CYCLE_LEFT
    # To far left
    else:
      turnDutyCycle = DUTY_CYCLE_RIGHT
      
  elif STATE[0] == RIGHT:
    if potValue >= (MAX_RIGHT_TURN_VALUE - TURN_TOLERANCE) and potValue <= (MAX_RIGHT_TURN_VALUE + TURN_TOLERANCE):
      turnDutyCycle = 0.0
    # To far left
    elif potValue < (MAX_RIGHT_TURN_VALUE - TURN_TOLERANCE):
      turnDutyCycle = DUTY_CYCLE_RIGHT
    # To far right
    else:
      turnDutyCycle = DUTY_CYCLE_LEFT

  turnMotorSignal.ChangeDutyCycle(turnDutyCycle)

#-------------------------------------------------------------------------------
def changeStates():
  '''
  Change the state of the vehicle
  '''
  global STATE, stateCount
  if (stateCount % STATE_FREQ) == 0:
    # Turn Diretion
    if STATE[0] == STRAIGHT:
      STATE[0] = LEFT
    elif STATE[0] == LEFT:
      STATE[0] = RIGHT
    elif STATE[0] == RIGHT:
      STATE[0] = STRAIGHT

    # Velocity Diretion
    if STATE[1] == SLOW:
      STATE[1] = FAST
    elif STATE[1] == FAST:
      STATE[1] = SLOW

  stateCount += 1

#-------------------------------------------------------------------------------
def consumeSerialData(ser):
  '''
  Consumes the data off the serial line that the Arduino sends
  @param ser - the serial communication line
  @return left wheel velocity
  @return right wheel velocity
  @return pot steering value
  '''
  leftVelocity = -1.0
  rightVelocity = -1.0
  potValue = -1.0
  for i in range(3):
    header, value = parseSerialLine(ser.readline().strip().decode('ascii'))
    if header == 'LV':
      leftVelocity = value
    elif header == 'RV':
      rightVelocity = value
    elif header == 'S':
      potValue = value
  
  return leftVelocity, rightVelocity, potValue

#-------------------------------------------------------------------------------
def parseSerialLine(line):
  '''
  Parses the incoming serial line
  @param line - the consumed serial line
  @return the line header
  @return the decimal value
  '''
  header = None
  value = None
  if line:
    delIndex = line.find(':')
    if delIndex >= 0:
      header = line[ : delIndex ]
      value = float(line[ (delIndex + 1) : ])
  return header, value

#-------------------------------------------------------------------------------
def setup():
  '''
  Setup the Raspberry Pi to run
  @return the serial communication line
  @return the drive motor signal
  @return the turn motor signal
  '''
  ser = serial.Serial(ARDUINO_SERIAL_PORT, BAUDRATE)
  ser.baudrate = BAUDRATE
  GPIO.setwarnings(False)
  GPIO.setmode(GPIO.BOARD)

  # Setup the drive motor
  GPIO.setup(DRIVE_PIN, GPIO.OUT)
  driveMotorSignal = GPIO.PWM(DRIVE_PIN, HBRIDGE_MOTOR_FREQ)
  driveMotorSignal.start(driveDutyCycle)

  # Setup the turn motor
  GPIO.setup(TURN_PIN, GPIO.OUT)
  turnMotorSignal = GPIO.PWM(TURN_PIN, HBRIDGE_MOTOR_FREQ)
  turnMotorSignal.start(turnDutyCycle)

  return ser, driveMotorSignal, turnMotorSignal

#-------------------------------------------------------------------------------
if __name__ == '__main__':
  main()

