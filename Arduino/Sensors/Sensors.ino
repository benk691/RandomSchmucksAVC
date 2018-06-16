#include "Constants.hh"
#include "Potentiometer.hh"

Potentiometer frontPotSensor(Constants::FRONT_POT_PIN);

/**
 * Setup the Arduino and the sensors
 */
void setup() 
{
  Serial.begin(Constants::BAUDRATE);
  frontPotSensor.setup();
}

/**
 * Run the Arduino continuously
 */
void loop() 
{
}

