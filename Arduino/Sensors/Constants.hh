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

  // The Front steering potentiometer pin (this is an Analog pin)
  static const unsigned int FRONT_POT_PIN = 2;
};

#endif // CONSTANTS_HH