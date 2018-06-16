
class Motor:
  '''
  Controls the motor
  '''
  #-------------------------------------------------------------------------------
  def __init__(self, GPIO, pin, freq, dutyCycle):
    '''
    Initializes the basics of the moto. This starts the PWM cycle on the pin
    @param GPIO - the current instance of the RPi.GPIO import that is being used
    @param pin - the pin the motor is on
    @param freq - the PWM frequency to use
    @param dutyCycle - the PWM duty cycle to use
    '''
    self.pin = pin
    self.freq = freq
    self.dutyCycle = dutyCycle
    self.signal = GPIO.PWM(self.pin, self.freq)
    self.signal.start(self.dutyCycle)

  #-------------------------------------------------------------------------------
  def changeDutyCycle(self, dutyCycle):
    '''
    Changes the motor's duty cycle
    @param dutyCycle - the duty cyle to use
    '''
    self.signal.changeDutyCycle(dutyCycle)
    self.dutyCycle = dutyCycle

  #-------------------------------------------------------------------------------
  def changeFrequency(self, freq):
    '''
    Changes the motor's frequency
    @param freq - the freq to use
    '''
    self.signal.changeFrequency(freq)
    self.freq = freq

  #-------------------------------------------------------------------------------
  def _debugDescription():
    '''
    Generates debugging information about hte motor
    @return string describing debug information
    '''
    desc += "Motor Info:\n"
    desc += "\tpin = {0}\n".format(self.pin)
    desc += "\tfreq = {0}\n".format(self.freq)
    desc += "\tdutyCycle = {0}\n".format(self.dutyCycle)
    return desc

  #-------------------------------------------------------------------------------
  def __repr__(self):
    '''
    Gets the string representation of the class
    @return string representation of the class
    '''
    return self._debugDescription()

  #-------------------------------------------------------------------------------
  def __str__(self):
    '''
    Gets the string representation of the class
    @return string representation of the class
    '''
    return self._debugDescription()

  #-------------------------------------------------------------------------------
  def __del__(self):
    '''
    Destructor
    '''
    self.signal.stop()

