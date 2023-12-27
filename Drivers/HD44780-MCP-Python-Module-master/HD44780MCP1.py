#!/usr/bin/env python3

"""HD44780-MCP, module for use with a HD44780 LCD driver module connected to
an MCP23008/17 io expander chip

created June 7, 2020 OM
modified June 18, 2020 OM"""

"""
Copyright 2020 Owain Martin

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import time

class HD44780:

    def __init__(self, mcp, rs, rw, e, dbList = [], rows = 1, characters = 16, mode = 0, font = 0):

        self.rsPin = rs         # register select pin

        if rw < 0:
            self.rwPin = None   # read/write set toNone, the pin will be ignored
        else:
            self.rwPin = rw     # read/write pin
        self.ePin = e           # enable pin

        self.mode = mode        # 1 = 8 bit, 0 = 4 bit

        # LCD number of rows
        if rows == 1:
            self.rows = 0       # variable used in function set command
            self.numRows = 1    # number of actual rows of LCD screen
        else:
            self.rows = 1       # variable used in function set command
            if (rows != 2 or rows !=4):
                self.numRows = 4    # number of actual rows of LCD screen
            else:
                self.numRows = rows # number of actual rows of LCD screen
            

        self.numCharacters = characters # LCD number of characters per row            
        self.font = font  # 0: 5x8 dot characters, 1: 5x10 dot characters
        self.timeDelay = 0.01

        self.displayOn = True
        self.cursorOn = True
        self.blinkOn = False
        self.ID = 1
        self.displayShift = False

        if self.mode == 0:
            # 4 bit Databus pin assignments
            self.db4Pin = dbList[0]
            self.db5Pin = dbList[1]
            self.db6Pin = dbList[2]
            self.db7Pin = dbList[3]
        elif self.mode == 1:
            # 8 bit Databus pin assignments
            self.db0Pin = dbList[0]
            self.db1Pin = dbList[1]
            self.db2Pin = dbList[2]
            self.db3Pin = dbList[3]
            self.db4Pin = dbList[4]
            self.db5Pin = dbList[5]
            self.db6Pin = dbList[6]
            self.db7Pin = dbList[7]        

        self.MCP = mcp       

        self.MCP.set_mode(self.rsPin, 'output')
        if self.rwPin != None:
            self.MCP.set_mode(self.rwPin, 'output')
        self.MCP.set_mode(self.ePin, 'output')

        if self.mode == 1:
            self.MCP.set_mode(self.db0Pin, 'output')
            self.MCP.set_mode(self.db1Pin, 'output')
            self.MCP.set_mode(self.db2Pin, 'output')
            self.MCP.set_mode(self.db3Pin, 'output')
            
        self.MCP.set_mode(self.db4Pin, 'output')
        self.MCP.set_mode(self.db5Pin, 'output')
        self.MCP.set_mode(self.db6Pin, 'output')
        self.MCP.set_mode(self.db7Pin, 'output')

        self.initialization_sequence()

        return


    def initialization_sequence(self):
        """initiazation sequence, sequence to initialize the module"""

        if self.mode == 0:
            # 4 bit initialization

            # the following 3 set_pins help with
            # initializing the screen after it has already
            # been initialized. Without, unintended settings
            # are set
            self.set_pins4(0,0,[0,0,1,1])
            time.sleep(0.05)

            self.set_pins4(0,0,[0,0,1,1])
            time.sleep(0.01)

            self.set_pins4(0,0,[0,0,1,1])
            time.sleep(0.01)

            # set module to 4 bit operation
            # DB7 - DB5 of 001 for function set command
            # DB4 sets operation to 4 bit
            self.set_pins4(0,0,[0,0,1,0])

            # DB7 - DB5 of 001 for function set command
            # DB4 sets operation to 4 bit            
            # DB3 sets number of lines/rows to 1 or 2
            # DB2 sets font type to 5x8 dot or 5x10 dot
            self.set_pins(0,0,[0,0,1,0, self.rows, self.font,0,0])            

        else:
            # 8 bit initialization
            # DB7 - DB5 of 001 for function set command
            # DB4 sets operation to 8 bit            
            # DB3 sets number of lines/rows to 1 or 2
            # DB2 sets font type to 5x8 dot or 5x10 dot
            self.set_pins(0,0,[0,0,1,1, self.rows, self.font,0,0]) 
            pass

        # set entry mode        
        self.set_entry_mode(self.ID, self.displayShift)

        # clear display
        self.clear_display()          

        # set display on, cursor on and blinking off            
        self.set_display(self.displayOn,self.cursorOn,self.blinkOn)

        return

    def display_character(self, char):

        if isinstance(char, str):
            char = ord(char)
        elif isinstance(char, int):
            pass
        else:
            return    

        bitList = []

        for i in range(7,-1,-1):
            bitList.append((char>>i) & 0x1)        

        self.set_pins(1,0,bitList)

        return

    def display_string(self, msg):

        for letter in msg:
            self.display_character(letter)

        return

    def set_pins(self,rs, rw, dbList):
        """set_pins, function to transfer data to the HD44780 module
        from the pi

        dbList in format [DB7, DB6, DB5, DB4, DB3, DB2, DB1, DB0]
        DB3 to DB0 only required for 8 bit operation"""

        if self.mode == 1:

            self.MCP.output(self.rsPin, rs)
            if self.rwPin != None:
                self.MCP.output(self.rwPin, rw)
            self.MCP.output(self.ePin, True)

            # set DB7 - DB0
            self.MCP.output(self.db7Pin, dbList[0])
            self.MCP.output(self.db6Pin, dbList[1])
            self.MCP.output(self.db5Pin, dbList[2])  
            self.MCP.output(self.db4Pin, dbList[3])        
            self.MCP.output(self.db3Pin, dbList[4])
            self.MCP.output(self.db2Pin, dbList[5])
            self.MCP.output(self.db1Pin, dbList[6])  
            self.MCP.output(self.db0Pin, dbList[7])

            self.MCP.output(self.ePin, False)
            time.sleep(self.timeDelay)

        else:
            self.set_pins4(rs,rw,dbList[:4])
            self.set_pins4(rs,rw,dbList[4:])

        return

    def set_pins4(self,rs, rw, dbList):
        """set_pins, function to transfer 4 bits of data to the HD44780 module
        from the pi, used for initialization when using 4 bit mode

        dbList in format [DB7, DB6, DB5, DB4]"""

        self.MCP.output(self.rsPin, rs)
        if self.rwPin != None:
            self.MCP.output(self.rwPin, rw)
        self.MCP.output(self.ePin, True)

        # set DB7 - DB4
        self.MCP.output(self.db7Pin, dbList[0])
        self.MCP.output(self.db6Pin, dbList[1])
        self.MCP.output(self.db5Pin, dbList[2])  
        self.MCP.output(self.db4Pin, dbList[3])        

        self.MCP.output(self.ePin, False)
        time.sleep(self.timeDelay)

        return

    def clear_display(self):
        """clear_display, clears display and sets DDRAM address at 0
        in the address counter"""

        # DB7 to DB0 - 0000 0001
        self.set_pins(0,0,[0,0,0,0,0,0,0,1])

        return

    def return_home(self):
        """return_home, sets DDRAM address at 0 in the address 
        counter and returns display from being shifted to original position"""

        # DB7 to DB0 - 0000 0010
        # shift display back to original position
        # and set DDRAM address back to 0
        self.set_pins(0,0,[0,0,0,0,0,0,1,0])

        return

    def set_cursor(self, row, column):
        """set_cursor, function to set the current cursor
        position"""

        # DB7 set to 1 for set DDRAM address operations
        bitList = [1]

        if self.numRows == 1:
            if column > 80:
                column = 80           
        elif self.numRows == 2:
            if column > 40:
                column = 40            
            if row == 2:
                column = column + 0x40
        elif self.numRows == 4:
            if column > 20:
                column = 20
            if row == 2:
                column = column + 0x40
            if row == 3:
                column = column + 20
            if row == 4:
                column = column + 0x40 + 20

        column = column - 1                
        for i in range(6,-1,-1):
                bitList.append((column>>i) & 0x1)
        
        self.set_pins(0,0,bitList)        

        return               

    def set_display(self, on = 0, cursor = 0, blinking = 0):
        """set_display, function to set the display on/off, turn the
        cursor on/off and blinking on/off"""

        if (on == 1 or on == True):
            self.displayOn = True
        else:
            self.displayOn = False

        if (cursor == 1 or cursor == True):
            self.cursorOn = True
        else:
            self.cursorOn = False

        if (blinking == 1 or blinking == True):
            self.blinkOn = True
        else:
            self.blinkOn = False
            
        # DB7 - DB3 of 00001 for display set            
        # DB2 sets display on/off
        # DB1 sets cursor on/off
        # DB0 sets blinking on/off
        self.set_pins(0,0,[0,0,0,0,1,self.displayOn,self.cursorOn,self.blinkOn])

        return

    def set_entry_mode(self, ID = 1, displayShift = 0):
        """set_entry_mode, function to set the entry mode variable;
        increment/decrement DDRAM address and display shift on/off"""

        if (ID == 1 or ID == True):
            self.ID == 1
        else:
            self.ID == 0

        if (displayShift == 1 or displayShift == True):
            self.displayShift = True
        else:
            self.displayShift = False

        
        # set entry mode
        # DB 7 - DB2 of 000001 for entry mode set            
        # DB1 increment/decrement DDRAM address by 1
        # when a character is written/read to from DDRAM
        # DB0 display shift on/off - when DB0 1, DB1 1 - left, 0 - right
        self.set_pins(0,0,[0,0,0,0,0,1,self.ID,self.displayShift])

        return

    def blink(self, blink = True):
        """blink, function to turn blinking on and off"""

        self.set_display(self.displayOn, self.cursorOn, blink)

        return

    def cursor(self, cursor = True):
        """cursor, function to turn the cursor on and off"""

        self.set_display(self.displayOn, cursor, self.blinkOn)

        return

    def display(self, display = True):
        """display, function to turn the display on and off"""

        self.set_display(display, self.cursorOn, self.blinkOn)

        return

    def scroll_left(self, numSpaces = 1, delay = 0):
        """scroll_left, function to scroll entire display left """

        # move cursor and display around
        # does not change DDRAM
        # DB4 set to 1 for cursor/display shift command
        # DB3 - 1 Display shift, 0 Cursor shift
        # DB2 - 1 right shift, 0 left shift

        for i in range(0, numSpaces):
            self.set_pins(0,0,[0,0,0,1,1,0,0,0])
            time.sleep(delay)

        return

    def scroll_right(self, numSpaces = 1, delay = 0):
        """scroll_right, function to scroll entire display right """

        # move cursor and display around
        # does not change DDRAM
        # DB4 set to 1 for cursor/display shift command
        # DB3 - 1 Display shift, 0 Cursor shift
        # DB2 - 1 right shift, 0 left shift

        for i in range(0, numSpaces):
            self.set_pins(0,0,[0,0,0,1,1,1,0,0])
            time.sleep(delay)

        return

    def cursor_left(self, numSpaces = 1, delay = 0):
        """cursor_left, function to move the cursor left"""

        # move cursor and display around
        # does not change DDRAM
        # DB4 set to 1 for cursor/display shift command
        # DB3 - 1 Display shift, 0 Cursor shift
        # DB2 - 1 right shift, 0 left shift

        for i in range(0, numSpaces):
            self.set_pins(0,0,[0,0,0,1,0,0,0,0])
            time.sleep(delay)

        return

    def cursor_right(self, numSpaces = 1, delay = 0):
        """cursor_right, function to move the cursor right"""

        # move cursor and display around
        # does not change DDRAM
        # DB4 set to 1 for cursor/display shift command
        # DB3 - 1 Display shift, 0 Cursor shift
        # DB2 - 1 right shift, 0 left shift

        for i in range(0, numSpaces):
            self.set_pins(0,0,[0,0,0,1,0,1,0,0])
            time.sleep(delay)

        return

        


if __name__ == "__main__":

    import MCP230XX

    mode = 3    

    if mode == 1:
        # 16x2 LCD testing using discrete MCP23008 io expander chip

        # set up MCP        
        i2cAddr = 0x20  # MCP23008 i2c address

        MCP = MCP230XX.MCP230XX('MCP23008', i2cAddr)

        # initialize LCD
        LCD = HD44780(MCP, 1, -1, 2, [3,4,5,6], rows = 2, characters = 16, mode = 0, font = 0)
        
        time.sleep(2)
        LCD.display_string("glue")
        time.sleep(2)
        #LCD.set_display(0)   # turn display off
        LCD.display(False)
        time.sleep(2)
        LCD.display()
        #LCD.set_display(1,1) # turn display on with cursor
        #time.sleep(2)
        #LCD.set_cursor(1,16)
        #time.sleep(2)
        LCD.set_cursor(2,1)
        LCD.display_string("stick")
        time.sleep(2)
        LCD.blink()
        time.sleep(2)
        LCD.cursor(False)
        LCD.blink(False)
        time.sleep(2)
        LCD.cursor(True)
        LCD.scroll_right(2)
        time.sleep(2)
        LCD.scroll_left(3)
        time.sleep(2)
        LCD.scroll_right(1)
        time.sleep(2)
        LCD.cursor_left(2)
        time.sleep(2)
        LCD.cursor_right(2)
        
        """LCD.set_entry_mode(1,1)
        LCD.display_string(" in the mud!!")
        time.sleep(1)
        LCD.scroll_right(10)
        LCD.set_entry_mode(1,0)"""


    elif mode == 2:
        # 20x4 LCD 4 bit testing using Adafruit i2c LCD backpack (uses MCP23008)

        # set up MCP
        blPin = 7 # Back light pin when using Adafruit LCD backpack
        i2cAddr = 0x20  # MCP23008 i2c address

        MCP = MCP230XX.MCP230XX('MCP23008', i2cAddr)
        MCP.set_mode(blPin, 'output')
        MCP.output(blPin, True) # turn backlight on - for Adafruit LCD backpack use

        # initialize LCD
        LCD = HD44780(MCP, 1, -1, 2, [3,4,5,6], rows = 4, characters = 20, mode = 0, font = 0)        

        time.sleep(2)
        LCD.display_string("glue")
        time.sleep(1)
        LCD.set_cursor(2,1)
        LCD.display_string("stick")
        time.sleep(1)
        LCD.set_cursor(3,1)
        LCD.display_string("Four lines long")
        time.sleep(1)
        LCD.set_cursor(4,1)
        LCD.display_string("Fun times")
        time.sleep(1)

        LCD.blink()
        time.sleep(2)
        LCD.cursor(False)
        LCD.blink(False)
        time.sleep(2)
        LCD.cursor(True)
        LCD.display(False)
        time.sleep(2)
        LCD.display()

        LCD.scroll_right(3)
        time.sleep(2)
        LCD.scroll_left(23, 0.5)
        time.sleep(2)
        LCD.cursor_left(12, 0.3)
        time.sleep(2)
        LCD.cursor_right(12, 0.3)
        time.sleep(2)
        LCD.return_home()
        

    elif mode == 3:
        # 20x4 LCD 8 bit testing

        # set up MCP        
        i2cAddr = 0x20  # MCP23017 i2c address
        
        MCP = MCP230XX.MCP230XX('MCP23017', i2cAddr)

        # initialize LCD
        LCD = HD44780(MCP, 1, -1, 2, [3,4,5,6,8,9,10,11], rows = 4, characters = 20, mode = 1, font = 0)

        time.sleep(2)
        LCD.display_string("glue")
        time.sleep(1)
        LCD.set_cursor(2,1)
        LCD.display_string("stick")
        time.sleep(1)
        LCD.set_cursor(3,1)
        LCD.display_string("Four lines long")
        time.sleep(1)
        LCD.set_cursor(4,1)
        LCD.display_string("Fun times")
        time.sleep(1)

        LCD.blink()
        time.sleep(2)
        LCD.cursor(False)
        LCD.blink(False)
        time.sleep(2)
        LCD.cursor(True)
        LCD.display(False)
        time.sleep(2)
        LCD.display()
        LCD.scroll_left(2, 0.5)


    #LCD.clear_display()
    


        

            

        
