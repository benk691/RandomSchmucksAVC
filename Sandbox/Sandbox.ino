/***********************************************************************************
 * GENERAL CONSTANTS
 ***********************************************************************************/
// Baudrate
const int BAUDRATE = 9600;
// If the pot is at this value then the wheels are fully turned to the right
const int WHEELS_MAX_RIGHT = 1023;
// If the pot is at this value then the wheels are fully turned to the left
const int WHEELS_MAX_LEFT = 0;
// If the pot is at this value, then the vehicle is going straight
const int WHEELS_STRAIGHT = (abs(WHEELS_MAX_RIGHT - WHEELS_MAX_LEFT) / 2);

/***********************************************************************************
 * ANALOG PINS
 ***********************************************************************************/
// potentiometer pin
const int potPin = 2;
// tachometer pin
const int tachPin = 3;

/***********************************************************************************
 * SENSOR VALUES
 ***********************************************************************************/
// Value read in from the pot
int potVal = 0;
// Value read in from the tachometer
int tachVal = 0;

/***********************************************************************************
 * FUNCTIONS
 ***********************************************************************************/

const bool TEST_POT = false;
const bool TEST_TACH = false;
const bool TEST_ARDUINO_PI = false;

/**
 * Sets up the Arduino
 */
void setup() 
{
  Serial.begin(BAUDRATE);
}

/**
 * Runs the Arduino loop
 */
void loop() 
{
  if (TEST_POT)
  {
    // Read Potentiometer
    potVal = analogRead(potPin);
    // display the read in value
    Serial.print("Pot Value = ");
    Serial.println(potVal, DEC);
    delay(300);
  }

  if (TEST_TACH)
  {
    // Read Tachometer
    tachVal = analogRead(tachPin);
    // display the read in value
    Serial.print("Tach Value = ");
    Serial.println(tachVal, DEC);
    delay(300);
  }

  if (TEST_ARDUINO_PI)
  {
    const bool TEST_SERIAL = true;
    const bool TEST_I2C = false;
    String data = "Hello From Arduino";

    if (TEST_SERIAL)
    {
      Serial.println(data);
      delay(300);
    }
    else if (TEST_I2C)
    {
      
    }
  }
}
