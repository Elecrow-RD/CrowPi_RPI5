#!/usr/bin/python
# -*- coding: utf-8 -*-
# http://elecrow.com/

from gpiozero import Button,Buzzer
import time

# configure both button and buzzer pins
button = Button(26)
buzzer = Buzzer(18)



try:
    while True:
        if button.is_pressed:
            buzzer.on()
        else:
            buzzer.off()
except KeyboardInterrupt:
    buzzer.close()
