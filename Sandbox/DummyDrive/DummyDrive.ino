
// Center: 483
// Left Full: 605
// Right Full: 421

const unsigned int BAUDRATE = 9600;

// Pins
const unsigned int POT_PIN = 1;
const unsigned int RIGHT_TACH_PIN = 2;
const unsigned int LEFT_TACH_PIN = 3;

// Sleep time (milli seconds)
const int SLEEP_TIME = 3;

// Pot
double steeringPotValue = 0;

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

/**
 * Setup the Arduino and the sensors
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
  steeringPotValue = analogRead(POT_PIN);

  if (loopCount % 1 == 0)
  {
    prevRightTachValue = rightTachValue;
    prevLeftTachValue = leftTachValue;
  }
  
  rightTachValue = analogRead(RIGHT_TACH_PIN);
  leftTachValue = analogRead(LEFT_TACH_PIN);

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
    
    Serial.print("LV:");
    Serial.println(leftVelocity, 5);
    
    Serial.print("RV:");
    Serial.println(rightVelocity, 5);
    
    Serial.print("S:");
    Serial.println(steeringPotValue, 5);

    loopCount = 0;
    rightStripCount = 0;
    leftStripCount = 0;
  }

  delay(SLEEP_TIME);
}

