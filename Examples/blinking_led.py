#!/usr/bin/python
# -*- coding: utf-8 -*-
# http://elecrow.com/
from gpiozero import LED
import time

# define LED pin
led = LED(26)


try:
    while True:
        # turn on LED
        led.off()
        # Wait half a second
        time.sleep(0.2)
        # turn off LED
        led.on()
        # Wait half a second
        time.sleep(0.2)
except KeyboardInterrupt:
    # CTRL+C detected, cleaning and quitting the script
    led.close()
