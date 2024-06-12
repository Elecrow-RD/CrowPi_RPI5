#!/usr/bin/python
# -*- coding: utf-8 -*-
# http://elecrow.com/

from gpiozero import DigitalInputDevice as Tilt
import time


# define tilt pin
tilt_sensor = Tilt(22)

while True:
    # positive is tilt to left negative is tilt to right
    if tilt_sensor.value:
        print("[-] Left Tilt")
    else:
        print("[-] Right Tilt")
    time.sleep(1)
