#include "Constants.hh"
#include "Potentiometer.hh"
#include "Tachometer.hh"

Potentiometer frontPotSensor(Constants::FRONT_POT_PIN);
Tachometer rightTach(Constants::REAR_RIGHT_WHEEL_TACH_PIN);
Tachometer leftTach(Constants::REAR_LEFT_WHEEL_TACH_PIN);

/**
 * Setup the Arduino and the sensors
 */
void setup() 
{
  Serial.begin(Constants::BAUDRATE);
  frontPotSensor.setup();
  rightTach.setup();
  leftTach.setup();
}

/**
 * Run the Arduino continuously
 */
void loop() 
{
}

