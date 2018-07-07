
class Constants:
  '''
  Defines constants for the Rasberry Pi
  '''
  # Arduino Serial Port
  ARDUINO_SERIAL_PORT = '/dev/ttyACM0'
  
  # Baudrate of the program
  BAUDRATE = 9600
  
  # The pin for signal 1 to the H-Bridge this pin will control the rear wheel speed
  HBRIDGE_S1_DRIVE_PIN = 3

  # The pin for signal 2 to the H-Bridge this pin will control the servo motor to turn the vehicle
  HBRIDGE_S2_TURN_PIN = 5

  # Initial PWM frequency for the H-Bridge motors (Hz)
  HBRIDGE_MOTOR_FREQ = 5000
