#!/usr/bin/python3.5
import serial
import time
import RPi.GPIO as GPIO
from decimal import Decimal
from threading import Thread

#-------Constants----------
ARDUINO_SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 9600
HBRIDGE_S1_PIN = 12
HBRIDGE_S2_PIN = 18
DRIVE_PIN = HBRIDGE_S1_PIN
TURN_PIN = HBRIDGE_S2_PIN
HBRIDGE_MOTOR_FREQ = 5000
MAX_RIGHT_TURN_VALUE = 421
CENTER_TURN_VALUE = 483
MAX_LEFT_TURN_VALUE = 605
TURN_TOLERANCE = 10
DUTY_CYCLE_DEAD_ZONE = 50.0
DUTY_CYCLE_LEFT = DUTY_CYCLE_DEAD_ZONE + 5.0
DUTY_CYCLE_RIGHT = DUTY_CYCLE_DEAD_ZONE - 5.0
#--------------------------

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
#driveDutyCycle = DUTY_CYCLE_DEAD_ZONE
#turnDutyCycle = DUTY_CYCLE_DEAD_ZONE

velGoal = 0
turnGoal = 0

FORWARD = 650
STOP = 1380
REV = 2112

# TODO: Control slow down more gradually
leftVelocity = 0.0
rightVelocity = 0.0
potValue = 0.0
ser = None

#-------------------------------------------------------------------------------
def main():
  '''
  Main program
  '''
  global ser
  global potValue
  global leftVelocity
  global rightVelocity
  try:
    ser, driveMotorSignal, turnMotorSignal = setup()
    #driveMotorSignal.ChangeDutyCycle(100)
    GPIO.output(DRIVE_PIN, GPIO.LOW)
    GPIO.output(TURN_PIN, GPIO.LOW)
    MIN = 1340
    MAX = 1440
    speed = MIN
    direction = 1

    goalThread = Thread(target=recvGoal)
    consumerThread = Thread(target=consumeSerialData)
    goalThread.start()
    consumerThread.start()
    # avg velocities
    # goal velocities ( recv goals while running if possible)
    # error = goal - avgVel
    # valueToSend= currentSpeedValue + error * 100
    # sleep 100ms
    currentVelocity = 1380
    currentTurn = 0
    sendSignal(DRIVE_PIN, currentVelocity, 0.3, 50)
    while True:
      #print("speed = {0}".format(speed))
      ##controlSpeed(DRIVE_PIN, speed)
      #sendSignal(TURN_PIN, speed, 0.1, 50)
      ##time.sleep(1)
      #speed += (direction * 10)
      #if speed < MIN or speed > MAX:
      #  direction *= -1
      #leftVelocity, rightVelocity, potValue = consumeSerialData(ser)
      print('LV: {0}'.format(leftVelocity))
      print('RV: {0}'.format(rightVelocity))
      print('S: {0}'.format(potValue))
      currentVelocity = controlVelocity(currentVelocity, leftVelocity, rightVelocity)
      currentTurn = controlSteering(currentTurn, potValue)
      print('')
      
      '''
      print('driveDutyCycle: {0}'.format(driveDutyCycle))
      print('turnDutyCycle: {0}'.format(turnDutyCycle))
      print('STATE: {0}'.format(STATE))
      controlVelocity(driveMotorSignal, leftVelocity, rightVelocity)
      controlSteering(turnMotorSignal, potValue)
      changeStates()
      time.sleep(1)
      '''
      #pass
  finally:
    #driveMotorSignal.stop()
    #turnMotorSignal.stop()
    sendSignal(DRIVE_PIN, STOP, 1.0, 50)
    goalThread.join(timeout=10)
    GPIO.cleanup()

#-------------------------------------------------------------------------------
def controlVelocity(currentVelocity, leftVelocity, rightVelocity):
  '''
  Controls the vehicles velocity
  @param driveMotorSignal - the signal to control the drive speed
  @param leftVelocity - the velocity of the left wheel
  @param rightVelocity - the velocity of the right wheel
  '''
  velAvg = (leftVelocity + rightVelocity) / 2.0
  if currentVelocity < 1380:
    velAvg *= -1.0
  error = velGoal - velAvg
  print("vel error = {0}".format(error))
  print("velGoal = {0}".format(velGoal))
  newVelocity = currentVelocity - error * 50.0
  #sendSignal(DRIVE_PIN, newVelocity, 0.3, 50)
  time.sleep(100 * 1e-3)
  currentVelocity = newVelocity
  print("currentVelocity = {0}".format(currentVelocity))

  return currentVelocity

  #driveDutyCycle += 1.0

  #if driveDutyCycle >= 100.0:
  #  driveDutyCycle = DUTY_CYCLE_DEAD_ZONE

  #for dc in range(100):
  #  print('duty cycle: {0}'.format(dc))
  #  driveMotorSignal.ChangeDutyCycle(dc)
  #  time.sleep(3)
  #if STATE[1] == SLOW:
  #  if (driveDutyCycle + 1.0) <= 0.0 or driveDutyCycle >= 100.0:
  #    driveDutyCycle = DUTY_CYCLE_DEAD_ZONE + 5.0
  #  else:
  #    driveDutyCycle += 1.0
  #elif STATE[1] == FAST:
  #  if driveDutyCycle <= 0.0 or (driveDutyCycle + 10.0) >= 100.0:
  #    driveDutyCycle = DUTY_CYCLE_DEAD_ZONE + 15.0
  #  else:
  #    driveDutyCycle += 10.0
  #driveMotorSignal.ChangeDutyCycle(driveDutyCycle)

#-------------------------------------------------------------------------------
def controlSteering(currentTurn, potValue):
  '''
  Controls the steering of the vehicle
  @param turnMotorSignal - the signal that controls the turn speed
  @param potValue - the potentiometer analog value
  '''
  #global turnDutyCycle
  terror = turnGoal - potValue
  print("turn error = {0}".format(terror))
  print("turnGoal = {0}".format(turnGoal))
  newTurn = 1380 - terror * 5.0
  sendSignal(TURN_PIN, newTurn, 0.3, 50)
  time.sleep(100 * 1e-3)
  currentTurn = newTurn
  print("currentTurn = {0}".format(currentTurn))

  return currentTurn

  #if STATE[0] == STRAIGHT:
  #  if potValue >= (CENTER_TURN_VALUE - TURN_TOLERANCE) and potValue <= (CENTER_TURN_VALUE + TURN_TOLERANCE):
  #    turnDutyCycle = DUTY_CYCLE_DEAD_ZONE
  #  # To far right
  #  elif potValue < (CENTER_TURN_VALUE - TURN_TOLERANCE):
  #    turnDutyCycle = DUTY_CYCLE_LEFT
  #  # To far left
  #  else:
  #    turnDutyCycle = DUTY_CYCLE_RIGHT
  #elif STATE[0] == LEFT:
  #  if potValue >= (MAX_LEFT_TURN_VALUE - TURN_TOLERANCE) and potValue <= (MAX_LEFT_TURN_VALUE + TURN_TOLERANCE):
  #    turnDutyCycle = DUTY_CYCLE_DEAD_ZONE
  #  # To far right
  #  elif potValue < (MAX_LEFT_TURN_VALUE - TURN_TOLERANCE):
  #    turnDutyCycle = DUTY_CYCLE_LEFT
  #  # To far left
  #  else:
  #    turnDutyCycle = DUTY_CYCLE_RIGHT
  #    
  #elif STATE[0] == RIGHT:
  #  if potValue >= (MAX_RIGHT_TURN_VALUE - TURN_TOLERANCE) and potValue <= (MAX_RIGHT_TURN_VALUE + TURN_TOLERANCE):
  #    turnDutyCycle = DUTY_CYCLE_DEAD_ZONE
  #  # To far left
  #  elif potValue < (MAX_RIGHT_TURN_VALUE - TURN_TOLERANCE):
  #    turnDutyCycle = DUTY_CYCLE_RIGHT
  #  # To far right
  #  else:
  #    turnDutyCycle = DUTY_CYCLE_LEFT

  #turnMotorSignal.ChangeDutyCycle(turnDutyCycle)

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
def consumeSerialData():
  '''
  Consumes the data off the serial line that the Arduino sends
  @param ser - the serial communication line
  @return left wheel velocity
  @return right wheel velocity
  @return pot steering value
  '''
  global leftVelocity
  global rightVelocity
  global potValue
  global ser
  leftVelocity = -0.0
  rightVelocity = -0.0
  potValue = -0.0

  while True:
    lVal = 0
    rVal = 0
    pVal = 0
    line = "DEAD"
    for i in range(24):
      line = ser.readline()
      #print("line = {0}".format(line))
      header, value = parseSerialLine(line.strip().decode('ascii'))
      if header == 'LV':
        lVal = value
        #print("lVal = {0}".format(lVal))
      elif header == 'RV':
        rVal = value
        #print("rVal = {0}".format(rVal))
      elif header == 'S':
        pVal = value
        #print("pVal = {0}".format(pVal))
    leftVelocity = lVal
    rightVelocity = rVal
    potValue = pVal

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
  #GPIO.setmode(GPIO.BOARD)
  GPIO.setmode(GPIO.BCM)

  # Setup the drive motor
  dbg("DRIVE_PIN", DRIVE_PIN)
  GPIO.setup(DRIVE_PIN, GPIO.OUT)
  driveMotorSignal= None
  #driveMotorSignal = GPIO.PWM(DRIVE_PIN, HBRIDGE_MOTOR_FREQ)
  #driveMotorSignal.start(driveDutyCycle)

  # Setup the turn motor
  dbg("TURN_PIN", TURN_PIN)
  GPIO.setup(TURN_PIN, GPIO.OUT)
  turnMotorSignal = None
  #turnMotorSignal = GPIO.PWM(TURN_PIN, HBRIDGE_MOTOR_FREQ)
  #turnMotorSignal.start(turnDutyCycle)

  return ser, driveMotorSignal, turnMotorSignal


#-------------------------------------------------------------------------------
def sendSignal(pin, sleepTime, signalLength, timing):
  # Keep sending signal until a new speed is read in
  elapse = 0
  while elapse < signalLength:
    s = time.time()
    controlSpeed(pin, sleepTime)
    e = time.time()
    elapse += (e - s)
    time.sleep(abs(timing) * 10e-5)
    #print("Timing: {0}".format(timing))

#-------------------------------------------------------------------------------
def controlSpeed(pin, sleepTime):
  GPIO.output(pin, GPIO.HIGH)
  time.sleep(abs(sleepTime) * 10e-7)
  GPIO.output(pin, GPIO.LOW)

#-------------------------------------------------------------------------------
def dbg(name, value):
  print("DBG: {0} = {1}".format(name, value))


#-------------------------------------------------------------------------------
def recvGoal():
  # Modify so this is on demand
  global velGoal
  global turnGoal
  while True:
    #velGoal = float(input("Enter velocity goal: "))
    turnGoal = float(input("Enter turn goal: "))
    

#-------------------------------------------------------------------------------
if __name__ == '__main__':
  main()

