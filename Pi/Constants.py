import math

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

# The min distance we want the sensors to detect (meters)
DIST_MIN_DISTANCE = 0.0

# The length of the queue in the distance sensor we want to use to store values
DIST_QUEUE_LENGTH = 10

# The velocity filter A value
VELOCITY_FILTER_A = 0.9

# The steering filter A value
STEERING_FILTER_A = 0.9

# The max loop count in the SensorConversion class
MAX_LOOP_COUNT = 15.0

# Number of milliseconds in a second
MILLI_SEC_IN_SEC = 1000.0

# Number of tape strips on each wheel
TACH_TOTAL_STRIPS = 30.0

# Right Tachometer high value
TACH_RIGHT_THRESHOLD_HIGH = 10000

# Right Tachometer low value
TACH_RIGHT_THRESHOLD_LOW = 8000

# Left Tachometer high value
TACH_LEFT_THRESHOLD_HIGH = 8500

# Left Tachometer low value
TACH_LEFT_THRESHOLD_LOW = 6500

# X index in a coordinate list
X = 0

# Y index in a coordinate list
Y = 1

# Heading index in a coordinate list
HEADING = 2

# Weight index in a coordinate list
WEIGHT = 3

# Radius index in a coordinate list
RADIUS = 3

# Distance away from the waypoint we want to start checking if we are at waypoint or not
WAYPOINT_CHECK_DIST = 1.0

# Number of checks we want to perform t oconfirm the vehicle is at a waypoint or not
WAYPOINT_MAX_CHECKS = 3

# Slowest we ever want the vehicle to go
MIN_VEHICLE_VELOCITY = 0.5

# Fastest we ever want the vehicle to go
MAX_VEHICLE_VELOCITY = 1.8

# Velocity scale factor. Scaled to decrease speed when the vehicle's heading is off from the waypoint's heading. This uses a degree that we want to be within before we start speeding above min velocity
VELOCITY_SCALE_FACTOR = (MAX_VEHICLE_VELOCITY - MIN_VEHICLE_VELOCITY) / math.radians(45.0)

# Control Planner update rate (Hz)
CONTROL_UPDATE_RATE = 10.0

# Update rate sleep threshold
CONTROL_SLEEP_THRESHOLD = 1.0e-3

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

# Noise that is applied to account for sensor inaccuracies in the distance
DISTANCE_NOISE = 0.0

# Slippage noise
SLIP_NOISE = 0.0

# Contant to allow us to steer more sharply
CONTROL_STEERING_AGRESSION = 1.1

# Update rate for the particle filter (Hz)
PARTICLE_FILTER_UPDATE_RATE = 10

# Left Most Steering Angle
MAX_LEFT_STEERING_ANGLE = math.pi / 4.0

# Right Most Steering Angle
MAX_RIGHT_STEERING_ANGLE = -math.pi / 4.0

# The position of the left distance sensor on the vehicle (relative to the vehicle)
DIST_LEFT_SENSOR_POSITION = [0.5, 0.3]

# The orientation of the left distance sensor (relative to the vehicle)
DIST_LEFT_SENSOR_OREINTATION = math.radians(90.0)

# The position of the right distance sensor on the vehicle (relative to the vehicle)
DIST_RIGHT_SENSOR_POSITION = [0.5, -0.3]

# The orientation of the right distance sensor (relative to the vehicle)
DIST_RIGHT_SENSOR_OREINTATION = math.radians(-90.0)

