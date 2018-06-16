#ifndef TACHOMETER_HH
#define TACHOMETER_HH

/**
 * General class to read a tachometer
 */
class Tachometer
{
public:
  /**
   * Initializes the tachometer
   */
  Tachometer(unsigned int tachPin) : 
    TACH_PIN(tachPin)
  {}

  /**
   * Setup this tachometer by setting the pin to read input from the tachometer
   * NOTE: Call this in the main setup function
   */
  void setup()
  {
    pinMode(TACH_PIN, OUTPUT);
  }

  /**
   * Sets the value of the tachometer by reading the pin
   */
  void setTachValue()
  {
    tachValue = analogRead(TACH_PIN);
  }

  /**
   * Gets the last read value off of the tachometer
   * @return last value of the tachometer
   */
  int getTachValue() const
  {
    return tachValue;
  }

  /**
   * Reads the tachometer value off the pin
   * @return the tachometer value
   */
  int readTachValue()
  {
    return tachValue = analogRead(TACH_PIN);
  }

  /**
   * Converts class to a string
   * @return the string representation of the class
   */
  String toString() const
  {
    String str = String("Tachometer:\n");
    str += String("\ttachValue = ");
    str += String(tachValue, DEC);
    str += String('\n');
    return str;
  }

private:
  // Tachometer pin (NOTE: this is an analog pin)
  const unsigned int TACH_PIN;
  // Last value read off of the tachometer
  int tachValue = 0;

  // Deleted functionality
  Tachometer() = delete;
  Tachometer(const Tachometer &rhs) = delete;
  Tachometer& operator=(const Tachometer &rhs) = delete;
};

#endif // TACHOMETER_HH