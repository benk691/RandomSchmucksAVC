#ifndef POTENTIOMETER_HH
#define POTENTIOMETER_HH

/**
 * General class to read a potentiometer
 */
class Potentiometer
{
public:
  /**
   * Initializes the potentiometer
   */
  Potentiometer(unsigned int potPin) : 
    POT_PIN(potPin)
  {}

  /**
   * Setup this potentiometer by setting the pin to read input from the potentiometer
   * NOTE: Call this in the main setup function
   */
  void setup()
  {
    pinMode(POT_PIN, INPUT);
  }

  /**
   * Sets the value of the potentiometer by reading the pin
   */
  void setPotValue()
  {
    potValue = analogRead(POT_PIN);
  }

  /**
   * Gets the last read value off of the potentiometer
   * @return last value of the potentiometer
   */
  int getPotValue() const
  {
    return potValue;
  }

  /**
   * Reads the potentiometer value off the pin
   * @return the potentiometer value
   */
  int readPotValue()
  {
    return potValue = analogRead(POT_PIN);
  }

  /**
   * Converts class to a string
   * @return the string representation of the class
   */
  String toString() const
  {
    String str = String("Potentiometer:\n");
    str += String("\tpotValue = ");
    str += String(potValue, DEC);
    str += String('\n');
    return str;
  }

private:
  // Potentiometer pin (NOTE: this is an analog pin)
  const unsigned int POT_PIN;
  // Last value read off of the potentiometer
  int potValue = 0;

  // Deleted functionality
  Potentiometer() = delete;
  Potentiometer(const Potentiometer &rhs) = delete;
  Potentiometer& operator=(const Potentiometer &rhs) = delete;
};

#endif // POTENTIOMETER_HH