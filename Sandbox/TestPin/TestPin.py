#!/usr/bin/python3.5
import RPi.GPIO as GPIO

TEST_PIN = 6

GPIO.setmode(GPIO.BCM)

try:
  GPIO.setup(TEST_PIN, GPIO.OUT)

  while True:
    if GPIO.input(TEST_PIN):
      print("{0} is HIGH".format(TEST_PIN))
    else:
      print("{0} is LOW".format(TEST_PIN))

finally:
  GPIO.cleanup()
