#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import HD44780MCP
import MCP230XX
from mcpi.minecraft import Minecraft

# Define Minecraft

mc = Minecraft.create()

#initialize MCP
i2cAddr = 0x21  # MCP23008/17 i2c address
MCP = MCP230XX.MCP230XX('MCP23008', i2cAddr)
#MCP = MCP230XX.MCP230XX('MCP23017', i2cAddr)

# turn on backlight if using Adafruit i2c LCD backpack (uses MCP23008)
blPin = 7 # Back light pin when using Adafruit LCD backpack
MCP.set_mode(blPin, 'output')
MCP.output(blPin, True) # turn backlight on - for Adafruit LCD backpack use

# set 16 character x 2 row LCD screen without rw pin, 4 bit mode
LCD = HD44780MCP.HD44780(MCP, 1, -1, 2, [3,4,5,6], rows = 2, characters = 16, mode = 0, font = 0)



while True:
    # get player position
    x, y, z = mc.player.getPos()
    x, y, z = float(str(x)[:3]),float(str(y)[:3]),float(str(z)[:3])
    pos = (str(x)+", "+str(y)+", "+str(z))
    print(pos)
    LCD.display_string("Position:")
    LCD.set_cursor(2,1)             # move cursor to 2nd row, 1st position
    LCD.display_string(pos)
    time.sleep(1)
    LCD.clear_display()
