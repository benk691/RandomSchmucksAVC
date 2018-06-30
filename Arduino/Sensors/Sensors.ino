#include "Constants.hh"
#include "Potentiometer.hh"
#include "Tachometer.hh"

Potentiometer steeringPotSensor(Constants::STEERING_POT_PIN);
Tachometer rightTach(Constants::REAR_RIGHT_WHEEL_TACH_PIN);
Tachometer leftTach(Constants::REAR_LEFT_WHEEL_TACH_PIN);

int steeringPotValue = 0;
int rightTachValue = 0;
int leftTachValue = 0;

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
  steeringPotValue = steeringPotSensor.readPotValue();
  rightTachValue = rightTach.readTachValue();
  leftTachValue = leftTach.readTachValue();
  // Send data to Rasberry Pi
  Serial.print("SP:");
  Serial.print(steeringPotValue, DEC);
  Serial.print(',');
  Serial.print("RT:");
  Serial.print(rightTachValue, DEC);
  Serial.print(',');
  Serial.print("LT:");
  Serial.println(leftTachValue, DEC);
}

