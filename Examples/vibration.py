#!/usr/bin/python
# -*- coding: utf-8 -*-
# http://elecrow.com/

from gpiozero import OutputDevice as Vibration
import time

# define vibration pin
vibration_sensor = Vibration(27)

# turn on vibration
vibration_sensor.on()
# wait half a second
time.sleep(0.5)
# turn off vibration
vibration_sensor.off()

