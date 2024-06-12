#!/usr/bin/python
# -*- coding: utf-8 -*-
from gpiozero import DigitalInputDevice as Touch
from mcpi.minecraft import Minecraft
import time

# create Minecraft Object
mc = Minecraft.create()

# set touch pin
touch_sensor = Touch(17)

while True:
    if touch_sensor.value == True: # look for button press
        mc.player.setPos(0, 0, 0) # teleport player
        print("Teleported successfully!")
        time.sleep(0.5) # wait 0.5 seconds
