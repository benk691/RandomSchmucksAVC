#include "Constants.hh"
#include "Potentiometer.hh"
#include "Tachometer.hh"

Potentiometer steeringPotSensor(Constants::STEERING_POT_PIN);
Tachometer rightTach(Constants::REAR_RIGHT_WHEEL_TACH_PIN);
Tachometer leftTach(Constants::REAR_LEFT_WHEEL_TACH_PIN);

// Sleep time (milli seconds)
const int SLEEP_TIME = 3;

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

int leftHigh = 0;
int leftThresholdHigh = 200;
int leftThresholdLow = 180;

int rightHigh = 0;
int rightThresholdHigh = 250;
int rightThresholdLow = 230;

// Timer
double startTime = 0;
double elapsedTime = 0;
int fractional = 0;

// ACK Setup
int ack = 0;
/**
 * Setup the Arduino and the sensors
 */
void setup() 
{
  Serial.begin(Constants::BAUDRATE);
  steeringPotSensor.setup();
  rightTach.setup();
  leftTach.setup();
}

/**
 * Run the Arduino continuously
 */
void loop() 
{
  // TODO: Consider
  // ACK based
  // Print all bytes one line ata time, then it doesn't matter when I start
  // Header: Value method
  
  // Make sure Pi and Arduino are both ready
  while (ack != 0xAA)
  {
    Serial.write(0xDEAD);

    if (Serial.available() > 0)
    {
      ack = Serial.read();
    }
  }
  
  steeringPotValue = steeringPotSensor.readPotValue();

  if (loopCount % 1 == 0)
  {
    prevRightTachValue = rightTachValue;
    prevLeftTachValue = leftTachValue;
  }
  
  rightTachValue = rightTach.readTachValue();
  leftTachValue = leftTach.readTachValue();

  if (rightHigh == 0 &&  rightTachValue > rightThresholdHigh)
  {
    rightStripCount += 0.5;
    rightHigh = 1;
  }

  if (rightHigh == 1 &&  rightTachValue < rightThresholdHigh)
  {
    rightStripCount += 0.5;
    rightHigh = 0;
  }

  if (leftHigh == 0 &&  leftTachValue > leftThresholdHigh)
  {
    leftStripCount += 0.5;
    leftHigh = 1;
  }

  if (leftHigh == 1 &&  leftTachValue < leftThresholdHigh)
  {
    leftStripCount += 0.5;
    leftHigh = 0;
  }

  loopCount++;

  if (loopCount >= 300)
  {
    elapsedTime = millis() - startTime;
    startTime = millis();
    rightVelocity = ((rightStripCount / 30.0) * 0.36 * M_PI) / (elapsedTime / 1000.0); 
    leftVelocity = ((leftStripCount / 30.0) * 0.36 * M_PI) / (elapsedTime / 1000.0);

    
//    Serial.print("LV:");
//    Serial.print(leftVelocity, 5);
//    Serial.print(',');
//    Serial.print("RV:");
//    Serial.println(rightVelocity, 5);
//    
//    Serial.print("LT:");
//    Serial.print(leftTachValue, DEC);
//    Serial.print(',');
//    Serial.print("RT:");
//    Serial.println(rightTachValue, DEC);
//
//    Serial.print("LSC:");
//    Serial.print(leftStripCount, 2);
//    Serial.print(',');
//    Serial.print("RSC:");
//    Serial.println(rightStripCount, 2);
//    
//    Serial.print("Elapsed Time:");
//    Serial.println(elapsedTime, 5);
//    
//
//    Serial.println();

    loopCount = 0;
    rightStripCount = 0;
    leftStripCount = 0;
  }

  delay(SLEEP_TIME);
}

