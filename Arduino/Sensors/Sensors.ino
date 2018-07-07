#include "Constants.hh"
#include "Potentiometer.hh"
#include "Tachometer.hh"

//Potentiometer steeringPotSensor(Constants::STEERING_POT_PIN);
Tachometer rightTach(Constants::REAR_RIGHT_WHEEL_TACH_PIN);
Tachometer leftTach(Constants::REAR_LEFT_WHEEL_TACH_PIN);

// Sleep time (milli seconds)
const int SLEEP_TIME = 10;

int steeringPotValue = 0;

// Tachometer Specific Data
const int TACH_THRESHOLD = 40;
int rightTachValue = 0;
int leftTachValue = 0;
int prevRightTachValue = 0;
int prevLeftTachValue = 0;
double rightVelocity = 0;
double rightStripCount = 0;
double leftVelocity = 0;
double leftStripCount = 0;
int loopCount = 0;

/**
 * Setup the Arduino and the sensors
 */
void setup() 
{
  Serial.begin(Constants::BAUDRATE);
//  steeringPotSensor.setup();
  rightTach.setup();
  leftTach.setup();
}

/**
 * Run the Arduino continuously
 */
void loop() 
{
//  steeringPotValue = steeringPotSensor.readPotValue();

  if (loopCount % 3 == 0)
  {
    prevRightTachValue = rightTachValue;
    prevLeftTachValue = leftTachValue;
  }
  
  rightTachValue = rightTach.readTachValue();
  leftTachValue = leftTach.readTachValue();

  if ((prevRightTachValue - rightTachValue) > TACH_THRESHOLD)
  {
    rightStripCount += 0.75;
  }

  if ((prevRightTachValue - rightTachValue) < -TACH_THRESHOLD)
  {
    rightStripCount += 0.25;
  }

  if ((prevLeftTachValue - leftTachValue) > TACH_THRESHOLD)
  {
    leftStripCount += 0.75;
  }

  if ((prevLeftTachValue - leftTachValue) < -TACH_THRESHOLD)
  {
    leftStripCount += 0.25;
  }

  loopCount++;

  if (loopCount >= 100)
  {
    rightVelocity = ((rightStripCount / 15.0) * 0.36 * M_PI) / 1.0;
    leftVelocity = ((leftStripCount / 15.0) * 0.36 * M_PI) / 1.0;
    loopCount = 0;
    rightStripCount = 0;
    leftStripCount = 0;
    Serial.print("LV:");
    Serial.print(leftVelocity, 5);
    Serial.print(',');
    Serial.print("RV:");
    Serial.println(rightVelocity, 5);

    Serial.print("LT:");
    Serial.print(leftTachValue, DEC);
    Serial.print(',');
    Serial.print("RT:");
    Serial.println(rightTachValue, DEC);
  }

  delay(SLEEP_TIME);

  
  // Send data to Rasberry Pi
//  Serial.print("SP:");
//  Serial.print(steeringPotValue, DEC);
//  Serial.print(',');
//  Serial.print("RT:");
//  Serial.print(rightTachValue, DEC);
//  Serial.print(',');
//  Serial.print("LT:");
//  Serial.println(leftTachValue, DEC);
}

