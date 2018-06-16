/***********************************************************************************
 * GENERAL CONSTANTS
 ***********************************************************************************/
// Baudrate
const int BAUDRATE = 9600;

/***********************************************************************************
 * ANALOG PINS
 ***********************************************************************************/
// Potentiometer pin
const int potPin = 2;
// Tachometer pin
const int tachPin = 3;

/***********************************************************************************
 * SENSOR VALUES
 ***********************************************************************************/
// Value read in from the pot
int potVal = 0;
// Value read in from the tachometer
int tachVal = 0;

/**
 * Setup the Arduino
 */
void setup() 
{
  Serial.begin(BAUDRATE);
}

/**
 * Run the Arduino continuously
 */
void loop() 
{
}
