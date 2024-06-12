#!/usr/bin/python
# -*- coding: utf-8 -*-
# http://elecrow.com/

from gpiozero import OutputDevice as Relay
import time
# define relay pin
relay = Relay(21)


# Open Relay
relay.on()
# Wait half a second
time.sleep(0.5)
# Close Relay
relay.off()
# Wait half a second
time.sleep(0.5)
# Close Relay


