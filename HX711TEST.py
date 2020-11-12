#!/usr/bin/env python3
from hx711 import HX711
from time import sleep
import RPi.GPIO as GPIO

while True:
    try:
        hx711 = HX711(
            dout_pin=5,
            pd_sck_pin=6,
            channel='A',
            gain=64
        )

        hx711.reset()   # Before we start, reset the HX711 (not obligate)
        measures = hx711.get_raw_data(num_measures=2)
    finally:
        GPIO.cleanup()  # always do a GPIO cleanup in your scripts!

    print("\n".join(measures))
    sleep(.001)
