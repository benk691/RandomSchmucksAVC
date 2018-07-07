#ifndef CONSTANTS_HH
#define CONSTANTS_HH

/**
 * Defines general constants used by the Arduino
 */
class Constants
{
public:
  // Baudrate of the program
  static const unsigned int BAUDRATE = 9600;

  // The steering potentiometer pin (this is an Analog pin)
  static const unsigned int STEERING_POT_PIN = 1;

  // The rear right wheel tachometer pin (this is an Analog pin)
  static const unsigned int REAR_RIGHT_WHEEL_TACH_PIN = 2;

  // The rear left wheel tachometer pin (this is an Analog pin)
  static const unsigned int REAR_LEFT_WHEEL_TACH_PIN = 3;
};

#endif // CONSTANTS_HH
