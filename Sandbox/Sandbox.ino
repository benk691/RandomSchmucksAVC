#include <Servo.h>
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
// Sleep time (milli seconds)
const int SLEEP_TIME = 300;

/***********************************************************************************
 * PWM PINS
 ***********************************************************************************/
const int servoPin = 9;

/***********************************************************************************
 * ANALOG PINS
 ***********************************************************************************/
// potentiometer pin
const int potPin = 2;
// tachometer pin
const int leftTachPin = 2;
const int rightTachPin = 2;

/***********************************************************************************
 * SENSOR VALUES
 ***********************************************************************************/
// Value read in from the pot
int potVal = 0;
// Value read in from the tachometer
int leftTachVal = 0;
int rightTachVal = 0;

/***********************************************************************************
 * FUNCTIONS
 ***********************************************************************************/

const bool TEST_POT = false;
const bool TEST_TACH = false;
const bool TEST_SERVO = false;
const bool TEST_IMU = false;
const bool TEST_ARDUINO_PI = false;

Servo servo;

/**
 * Sets up the Arduino
 */
void setup() 
{
  Serial.begin(BAUDRATE);

  if (TEST_SERVO)
  {
    servo.attach(servoPin);
  }
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
    delay(SLEEP_TIME);
  }

  if (TEST_TACH)
  {
    // Read Tachometer
    leftTachVal = analogRead(leftTachPin);
    rightTachVal = analogRead(rightTachPin);
    // display the read in value
    Serial.print("Left Tach Value = ");
    Serial.println(leftTachVal, DEC);
    Serial.print("Right Tach Value = ");
    Serial.println(rightTachVal, DEC);
    delay(SLEEP_TIME);
  }

  if (TEST_SERVO)
  {
    // Striaght
    servo.write(90);
    delay(SLEEP_TIME);
    // Direction 1
    servo.write(0);
    // Striaght
    servo.write(90);
    delay(SLEEP_TIME);
    // Direction 2
    servo.write(180);
    delay(SLEEP_TIME);
  }

  if (TEST_IMU)
  {
    // PASS
  }

  if (TEST_ARDUINO_PI)
  {
    const bool TEST_SERIAL = true;
    const bool TEST_I2C = false;
    String data = "Hello From Arduino";

    if (TEST_SERIAL)
    {
      Serial.println(data);
      delay(SLEEP_TIME);
    }
    else if (TEST_I2C)
    {
      
    }
  }
}
