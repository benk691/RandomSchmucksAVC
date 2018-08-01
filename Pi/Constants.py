
class Constants:
  '''
  Defines constants for the Rasberry Pi
  '''
  # Baudrate of the program
  BAUDRATE = 9600
  
  # ADC data/sample rate
  #   Valid Dat Rates and their mappings per the Adafrutii_Python_ADS1x15 repo
  #     8   ->  0x00
  #     16  ->  0x20
  #     32  ->  0x40
  #     64  ->  0x60
  #     128 ->  0x80
  #     250 ->  0xA0
  #     475 ->  0xC0
  #     860 ->  0xE0
  ADC_DATA_RATE = 860
