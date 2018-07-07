
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
    GPIO.setup(self.pin, GPIO.OUT)
    self.signal = GPIO.PWM(self.pin, self.freq)
    self.signal.start(self.dutyCycle)
    self.tabs = 0

  #-------------------------------------------------------------------------------
  def changeDutyCycle(self, dutyCycle):
    '''
    Changes the motor's duty cycle
    @param dutyCycle - the duty cyle to use
    '''
    self.signal.ChangeDutyCycle(dutyCycle)
    self.dutyCycle = dutyCycle

  #-------------------------------------------------------------------------------
  def changeFrequency(self, freq):
    '''
    Changes the motor's frequency
    @param freq - the freq to use
    '''
    self.signal.ChangeFrequency(freq)
    self.freq = freq

  #-------------------------------------------------------------------------------
  def setTabs(self, tabs):
    '''
    Set the number of tabs to use when printing out information
    @param tabs - the number of tabs to use in print out
    '''
    self.tabs = tabs

  #-------------------------------------------------------------------------------
  def _debugDescription(self):
    '''
    Generates debugging information about hte motor
    @return string describing debug information
    '''
    desc = "{0}Motor Info:\n".format('\t' * self.tabs)
    desc += "{0}\tpin = {1}\n".format('\t' * self.tabs, self.pin)
    desc += "{0}\tfreq = {1}\n".format('\t' * self.tabs, self.freq)
    desc += "{0}\tdutyCycle = {1}\n".format('\t' * self.tabs, self.dutyCycle)
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

