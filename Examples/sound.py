#!/usr/bin/python
# -*- coding: utf-8 -*-
# http://elecrow.com/

from gpiozero import DigitalInputDevice as Sound
import time

# define sound pin
sound_sensor = Sound(24)

while True:
    # check if sound detected or not
    if(sound_sensor.value == 0):
        print('Sound Detected')
        time.sleep(0.1)

