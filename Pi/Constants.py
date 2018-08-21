
class Constants:
  '''
  Defines constants for the Rasberry Pi
  '''
  # ADC data/sample rate
  #   Valid Data Rates and their mappings per the Adafruit_Python_ADS1x15 repo
  #     8   ->  0x00
  #     16  ->  0x20
  #     32  ->  0x40
  #     64  ->  0x60
  #     128 ->  0x80
  #     250 ->  0xA0
  #     475 ->  0xC0
  #     860 ->  0xE0
  ADC_DATA_RATE = 860

  # ADC gain value
  ADC_GAIN = 1

  # The ADC channel the potentiometer is on
  ADC_POT_CHNL = 0

  # The ADC channel the right wheel tachometer is on
  ADC_RIGHT_WHEEL_CHNL = 1

  # The ADC channel the left wheel tachometer is on
  ADC_LEFT_WHEEL_CHNL = 2

  # The frequency of the PWM sensor, this is the frequency used by the driving and steering motors
  PWM_SENSOR_FREQ = 120

  # The PWM channel the drive motor is on
  PWM_DRIVE_CHNL = 14

  # The PWM channel the turn motor is on
  PWM_TURN_CHNL = 15

  # Left distance sensor echo pin (this is a GPIO pin #)
  DIST_LEFT_ECHO_PIN = 27

  # Left distance sensor trigger pin (this is a GPIO pin #)
  DIST_LEFT_TRIGGER_PIN = 17

  # Left distance sensor echo pin (this is a GPIO pin #)
  DIST_RIGHT_ECHO_PIN = 22

  # Left distance sensor trigger pin (this is a GPIO pin #)
  DIST_RIGHT_TRIGGER_PIN = 18

  # The max distance we want the sensors to detect (meters)
  DIST_MAX_DISTANCE = 2.0

  # The length of the queue in the distance sensor we want to use to store values
  DIST_QUEUE_LENGTH = 10

  # X index in a coordinate list
  X = 0

  # Y index in a coordinate list
  Y = 1

  # Heading index in a coordinate list
  HEADING = 2

  # Weight index in a coordinate list
  WEIGHT = 3

  #--------------------------------
  #         Particle Filter
  #--------------------------------
  # Number of particles to generate for the particle filter
  PARTICLE_NUMBER = 10

  # Minimum particle number that we do not want to go below
  MIN_PARTICLE_NUMBER = 2

  # Length between the two axles on the vehicle (meters)
  VEHICLE_AXLE_LEN = 0.0

  # Two Points that define the start box, on the map, for the vehicle
  MAP_START_BOX = [ [0.0, 0.0], [0.0, 0.0] ]

  # The starting heading range for the vehicle
  MAP_HEADING_RANGE = [ 0.0, 0.0 ]

  # Offset that is applied so we can set 0 degrees with out worrying about North/South
  MAP_HEADING_OFFSET = 0.0

  # Noise that is applied to account for sensor inaccuracies in the heading
  HEADING_NOISE = 0.0

  # Noise that is applied to account for sensor inaccuracies in the steering angle
  STEERING_ANGLE_NOISE = 0.0

  # Noise that is applied to account for sensor inaccuracies in the velocity
  VELOCITY_NOISE = 0.0

  # Update rate for the particle filter (Hz)
  PARTICLE_FILTER_UPDATE_RATE = 10
  
  # Left Most Steering Angle
  MAX_LEFT_STEERING_ANGLE = 180

  # Right Most Steering Angle
  MAX_RIGHT_STEERING_ANGLE = 0

