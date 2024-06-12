#!/usr/bin/python
# -*- coding: utf-8 -*-
# http://elecrow.com/
from  gpiozero import DistanceSensor
import time


distancesensor = DistanceSensor(echo = 12,trigger = 16, max_distance = 5)

while True:
    print('Distance:',distancesensor.distance * 100,"cm")
    time.sleep(1)
