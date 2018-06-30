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
const int SLEEP_TIME = 10;

const int TACH_MAX = 112;
const int TACH_MIN = 50;
const int TACH_THRESHOLD = 40;

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
const int rightTachPin = 3;

/***********************************************************************************
 * SENSOR VALUES
 ***********************************************************************************/
// Value read in from the pot
int potVal = 0;
// Value read in from the tachometer
int leftTachVal = 0;
int rightTachVal = 0;
int prevRightTachVal = 0;

int onStrip = 0;
int offStrip = 0;
double stripCount = 0;
int rev = 0;
int revTime = 0;
int k1 = 0;
double velocity = 0;

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
//    time_t start_T;
//    time_t end_T;
//    if (onStrip == 0)
//    {
//       start_T = now();
//    }
    // Read Tachometer
//    if (k1 % 3 == 0)
      prevRightTachVal = rightTachVal;
    
    leftTachVal = analogRead(leftTachPin);
    rightTachVal = analogRead(rightTachPin);
    
//    Serial.print("rightTachVal = ");
//    Serial.println(rightTachVal, DEC);

//    Serial.print("rightTachVal - prevRightTachVal = ");
//    Serial.println(rightTachVal - prevRightTachVal, DEC);

    if ((prevRightTachVal - rightTachVal) > TACH_THRESHOLD)
    {
      stripCount += 0.75;
//      Serial.print("!~stripCount = ");
//      Serial.println(stripCount, DEC);
    }

    if ((prevRightTachVal - rightTachVal) < (-1 * TACH_THRESHOLD))
    {
      stripCount += 0.25;
    }
    else
    {
//      Serial.print("rightTachVal = ");
//      Serial.println(rightTachVal, DEC);
//      Serial.print("prevRightTachVal = ");
//      Serial.println(prevRightTachVal, DEC);
//      Serial.print("stripCount = ");
//      Serial.println(stripCount, DEC);
    }

    k1++;

    if (k1 >= 100)
    {
      Serial.print("stripCount = ");
      Serial.println(stripCount, DEC);
      velocity = ((stripCount / 15.0) * 0.36 * M_PI) / 1.0;
      k1 = 0;
      stripCount = 0;
      Serial.print("velocity = ");
      Serial.println(velocity, 5);
    }
    

//    if (rightTachVal >= (TACH_MAX + TACH_THRESHOLD) && 
//        rightTachVal >= (TACH_MAX - TACH_THRESHOLD))
//    {
//      onStrip++;
//      onStrip %= 15;
//    }
//
//    if (rightTachVal >= (TACH_MIN + TACH_THRESHOLD) && 
//        rightTachVal >= (TACH_MIN - TACH_THRESHOLD))
//    {
//      offStrip++;
//      offStrip %= 15;
//    }
//
//    if (onStrip == 0)
//    {
//      rev++;
//      end_T = now();
//      revTime = abs(minute(start_T) - minute(end_T)) * 60;
//      revTime += abs(second(start_T) - second(end_T));
//    }

//    Serial.print("Revolutions = ");
//    Serial.println(rev, DEC);
//    Serial.print("sec = ");
//    Serial.println(revTime, DEC);
    
    // display the read in value
    //Serial.print("Left Tach Value = ");
    //Serial.println(leftTachVal, DEC);
    //Serial.print("Right Tach Value = ");
    //Serial.println(rightTachVal, DEC);
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
