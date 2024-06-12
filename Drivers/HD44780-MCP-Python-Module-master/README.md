# HD44780-MCP-Python-Module
Python 3.x to use with the HD44780 LCD screen controlled with an MCP23008 or MCP230017 io expander chip which is
in turn controlled from a Raspberry Pi. Testing done using a 16x2 LCD and 20x4 LCD, both from Adafruit, MCP23008 and
MCP 230017 discrete DIP chips, an Adafruit i2C LCD backpack with onboard MCP23008 chip and a Pi3.

This requires the MCP230XX python module which I wrote, but I've include in this repository. The README file for that module
is found in the MCP230XX-Python-Module repository.

Also included is an example file which demonstrates some of the base functions.

Connections I used between the Pi and MCP23008 are as follows:

- Pi SCL to MCP SCL
- Pi SDA to MCP SDA
- Pi 3.3V to MCP VDD
- Pi Gnd to MCP Vss
- Pi 3.3V to MCP RESET (Could be tied to a Pi GPIO if you want active control of the RESET)
- Pi Gnd to MCP A0
- Pi Gnd to MCP A1
- Pi Gnd to MCP A2

With the above A0-A2 connections the i2c address is 0x20

Connections I used between the MCP23008 and LCD HD44780 are as follows:

- LCD Gnd to Pi Gnd
- LCD VDD to Pi 5V
- LCD V0 to middle pin of a 10k Potentiometer - Potentiometer required for contrast adjustment
- 10k Potentiometer outer leg 1 to Pi Gnd 
- 10k Potentiometer outer leg 2 to Pi 5V
- LCD RS to MCP GP1
- LCD RW to Pi Gnd
- LCD E to MCP GP2
- LCD DB0-DB3 - not connected
- LCD DB4 to MCP GP3
- LCD DB5 to MCP GP4
- LCD DB6 to MCP GP5
- LCD DB7 to MCP GP6
- LCD BL1 to Pi 5V - Backlight power
- LCD BL2 to 160 ohm resistor which connects to Pi Gnd

The above connections are for 4 bit mode operation

Current functions include:

- initialization - LCD = HD44780MCP.HD44780(mcp, rs, rw, e, dbList = [], rows = 1, characters = 16, mode = 0, font = 0)
- rs, rw, e are the pi pin numbers to connect to the LCD rs, rw & e pins
- rw is optional - if not using, assign to -1 and tie LCD rw pin to Pi Gnd
- dbList is a list of mcp pin numbers to connect to the LCD/HD44780 DB0-DB7 pins
- rows - number of rows on the LCD display, only 1, 2 & 4 are currently valid
- characters - number of characters per row on the LCD display
- mode - 0 for 4 bit mode, 1 for 8 bit mode
- font - 0 for 5x8 dot font, 1 for 5x10 dot font

- display_string(msg)
- set_cursor(row, column)
- blink(blink = True)
- cursor(cursor = True)
- display(display = True)
- scroll_left(numSpaces = 1, delay = 0)
- scroll_right(numSpaces = 1, delay = 0)
- cursor_left(numSpaces = 1, delay = 0)
- cursor_right(numSpaces = 1, delay = 0)
- clear_display()
- return_home()

- set_display(on = 0, cursor = 0, blinking = 0)
- set_entry_mode(ID = 1, displayShift = 0)

Notes:

The HD44780 can be connected in 4 or 8 bit modes. In 8 bit mode, all 8 of DB0-DB7 are used whereas in 4 bit mode
only DB4-DB7 are used.  If using 4 bit mode, only a list of the 4 pi pins used is required to be passed in the dbList
for initialization, see example below.

I haven't tried reading data back from the HD44780, if you try, consider using a logic level shifter as
the HD44780 will output 5V which could fry your pi, then you'd cry. As such if you don't plan on reading
info back from the HD44780 then there really is no need to use the RW pin.  

To not use the RW pin, connect the RW pin on the HD44780/LCD to Pi Gnd and assign it to -1 in the initialization
statement, i.e. LCD = HD44780MCP.HD44780(MCP, 1, -1, 2, [3,4,5,6], rows = 2, characters = 16, mode = 0, font = 0)

This module works with the Adafruit i2c LCD backpack (which are pretty nice). The backlight, when using the backpack,
is controlled using GP7 of the onboard MCP23008. The example file shows how to turn it on and off.

The HD44780 and MCP23008/17 data sheets are useful and can be downloaded from the Adafruit website and elsewhere.
