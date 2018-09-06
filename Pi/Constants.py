import math

# TODO: Go through all values and make sure we have the right numbers. -0 indicates that we have not decided on a number

'''
Defines constants for the Rasberry Pi
'''
#--------------------------------
#         PID Control
#--------------------------------
# Velocity proportional constant
VELOCITY_PID_P = 50.0

# Velocity integral constant
VELOCITY_PID_I = 100.0

# Velocity derivative constant
VELOCITY_PID_D = 0.0

# Velocity windup constant
VELOCITY_PID_WINDUP = 3.0

# Update rate of the velocity PID (Hz)
VELOCITY_PID_UPDATE_RATE = 10

# Steering angle proportional constant
STEERING_ANGLE_PID_P = 1.0

# Steering angle integral constant
STEERING_ANGLE_PID_I = 0.2

# Steering angle derivative constant
STEERING_ANGLE_PID_D = 0.0

# Steering angle windup constant
STEERING_ANGLE_PID_WINDUP = 3.0

# Update rate of the steering PID (Hz)
STEERING_PID_UPDATE_RATE = 40

# Wall follow proportional constant
WALL_FOLLOW_PID_P = 10000.0

# Wall follow integral constant
WALL_FOLLOW_PID_I = 0.0

# Wall follow derivative constant
WALL_FOLLOW_PID_D = 0.0

# Wall follow windup constant
WALL_FOLLOW_PID_WINDUP = 0.0

# Wall follow PID goal. Distance we want to keep away from the wall (meters)
WALL_FOLLOW_PID_DIST_GOAL = 1.5

# Wall follow update rate (Hz)
WALL_FOLLOW_UPDATE_RATE = 10.0
#---------------------------------

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
PWM_SENSOR_FREQ = 2000

# The PWM channel the drive motor is on
PWM_DRIVE_CHNL = 14

# The PWM channel the turn motor is on
PWM_TURN_CHNL = 15

# Max allowed pulse length
PWM_MAX_PULSE = 0.97

# Minimum allowed pulse length
PWM_MIN_PULSE = 0.0

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

# The steering median filter order value
STEERING_MEDIAN_FILTER_ORDER = 3

# The steering filter IIR A value
STEERING_IIR_FILTER_A = 0.9

# The velocity filter IIR A value
VELOCITY_IIR_FILTER_A = 0.9

# The heading filter IIR A value
HEADING_IIR_FILTER_A = 0.9

# The distance filter IIR A value
DIST_IIR_FILTER_A = 0.9

# Constant to apply to distance measurements if they are railed
DIST_MAX_FILTER = 0.3

# Number of milliseconds in a second
MILLI_SEC_IN_SEC = 1000.0

# Number of feet in a meter
FEET_IN_METER = 0.305

# Potentiometer value that keeps the vehicle straight
POT_STRAIGHT = 19203

# Potentiometer value of a max right turn
POT_RIGHT = 16400

# Potentiometer value of a max left turn
POT_LEFT = 22500

# Number of tape strips on each wheel
TACH_TOTAL_STRIPS = 30.0

# Right Tachometer high value
TACH_RIGHT_THRESHOLD_HIGH = 10000.0

# Right Tachometer low value
TACH_RIGHT_THRESHOLD_LOW = 8000.0

# Left Tachometer high value
TACH_LEFT_THRESHOLD_HIGH = 8500.0

# Left Tachometer low value
TACH_LEFT_THRESHOLD_LOW = 6500.0

# Length between the two axles on the vehicle (meters)
VEHICLE_AXLE_LEN = 0.9

# Diameter of the wheels on the vehicle (meters)
VEHICLE_WHEEL_DIAMETER = 0.36

# Wheel's Circumference (meters)
VEHICLE_WHEEL_CIRCUMFERENCE = 1.31

# Value to convert strip count to meters
STRIP_COUNT_TO_METERS = VEHICLE_WHEEL_CIRCUMFERENCE / TACH_TOTAL_STRIPS

# Steering pot value slope value for converting to a steering angle
STEERING_CONV_SLOPE = 0.00634

# Steering pot value y-intercept for converting to a steering angle
STEERING_Y_INTERCEPT = -124.13

# Distance away from the waypoint we want to start checking if we are at waypoint or not
WAYPOINT_CHECK_DIST = 1.0

# Number of checks we want to perform t oconfirm the vehicle is at a waypoint or not
WAYPOINT_MAX_CHECKS = 3

# Slowest we ever want the vehicle to go
MIN_VEHICLE_VELOCITY = 0.2

# Fastest we ever want the vehicle to go
MAX_VEHICLE_VELOCITY = 1.0

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
PARTICLE_NUMBER = 100

# Minimum particle number that we do not want to go below
MIN_PARTICLE_NUMBER = 0

# Two Points that define the start box, on the map, for the vehicle
MAP_START_BOX = [ [-0.25, -0.25], [0.25, 0.25] ]

# The starting heading range for the vehicle
MAP_HEADING_RANGE = [ math.radians(-7.0), math.radians(7.0) ]

# Offset that is applied so we can set 0 degrees with out worrying about North/South
MAP_HEADING_OFFSET = 0.0

# Noise that is applied to account for sensor inaccuracies in the heading
HEADING_NOISE = math.radians(3.0)

# Noise that is applied to account for sensor inaccuracies in the steering angle
STEERING_ANGLE_NOISE = math.radians(4.0)

# Noise that is applied to account for sensor inaccuracies in the velocity
VELOCITY_NOISE = 0.3

# Noise that is applied to account for sensor inaccuracies in the distance
DISTANCE_NOISE = 0.2

# Slippage noise
SLIP_NOISE = 0.1

# Contant to allow us to steer more sharply
CONTROL_STEERING_AGRESSION = 1.1

# Left Most Steering Angle
MAX_LEFT_STEERING_ANGLE = math.radians(25.0)

# Right Most Steering Angle
MAX_RIGHT_STEERING_ANGLE = math.radians(25.0)

# The position of the left distance sensor on the vehicle (relative to the vehicle)
DIST_LEFT_SENSOR_POSITION = [0.508, 0.381]

# The orientation of the left distance sensor (relative to the vehicle)
DIST_LEFT_SENSOR_ORIENTATION = math.radians(65.0)

# The position of the right distance sensor on the vehicle (relative to the vehicle)
DIST_RIGHT_SENSOR_POSITION = [0.508, -0.381]

# The orientation of the right distance sensor (relative to the vehicle)
DIST_RIGHT_SENSOR_ORIENTATION = math.radians(-65.0)

# Hading wrap around. Account for right hand rotation instead of left hand [degrees]
HEADING_WRAP_AROUND = 360.0

# Start Button Pin, Indicates that we are ready to start the program
START_BUTTON_PIN = 5

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

# Calibration file where we store are previously configured calibration settings
CALIB_SETTINGS_FILE = "/home/pi/Documents/RandomSchmucksAVC/calibrationSettings.info"

