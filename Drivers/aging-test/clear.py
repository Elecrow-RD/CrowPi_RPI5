#from rpi_ws281x import PixelStrip, Color
import CrowpiAllModule as pi
import HD44780MCP
import MCP230XX
#import RPi.GPIO as GPIO
from gpiozero import OutputDevice
from gpiozero import LED
from DFRobot_DHT20 import DFRobot_DHT20
from ht16k33segment_python import HT16K33Segment
import time
import re
from luma.led_matrix.device import max7219
from luma.core.interface.serial import spi, noop
from luma.core.render import canvas
from luma.core.virtual import viewport
from luma.core.legacy import text, show_message
from luma.core.legacy.font import proportional, CP437_FONT, TINY_FONT, SINCLAIR_FONT, LCD_FONT
import smbus

buzzer_pin = 18
# set type of the sensor
sensor = 11
# set pin number
vibration_pin = 27
realy_pin = 21


def seven_close():
    i2c = smbus.SMBus(1)
    segment = HT16K33Segment(i2c)
    segment.set_brightness(15)
    segment.clear()
    segment.update()


'''    
class LCDModule():

    def __init__(self):
        # Define LCD column and row size for 16x2 LCD.
        self.address = 0x21
        self.lcd_columns = 16
        self.lcd_rows = 2
        # Initialize the LCD using the pins
        self.lcd = LCD.Adafruit_CharLCDBackpack(address=self.address)

    def turn_off(self):
        # Turn backlight off
        self.lcd.set_backlight(1)

    def turn_on(self):
        # Turn backlight on
        self.lcd.set_backlight(0)

    def clear(self):
        # clear the LCD screen
        self.lcd.clear()

    def write_lcd(self,text):
        # turn on LCD
        self.turn_on()
        # wait 0.1 seconds
        time.sleep(0.05)
        # Print a two line message
        self.lcd.message(text)
        # wait 5 seconds
#         time.sleep(5)
        # clear screen
#         self.clear()
        # wait 0.1 seconds
        time.sleep(0.05)
        # turn off LCD
#         self.turn_off()
'''

'''
def RGB_close():


    LED_COUNT = 64        # Number of LED pixels.
    LED_PIN = 12          # GPIO pin connected to the pixels (18 uses PWM!).
    LED_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800khz)
    LED_DMA = 10          # DMA channel to use for generating signal (try 10)
    LED_BRIGHTNESS = 10  # Set to 0 for darkest and 255 for brightest
    LED_INVERT = False    # True to invert the signal (when using NPN transistor level shift)
    LED_CHANNEL = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53
    
    strip = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
    # Intialize the library (must be called once before other functions).
    strip.begin()
    

    for i in range(64):
        strip.setPixelColor(i, Color(0, 0, 0))
    strip.show()
'''
def LedMatrix_close():
    # create matrix device
    serial = spi(port=0, device=1, gpio=noop())
    device = max7219(serial, cascaded=1, block_orientation=90, rotate=0)
    device.hide()
    
    
    
    
#GPIO.setmode(GPIO.BCM)

#GPIO.setup(buzzer_pin, GPIO.OUT)
#GPIO.setup(vibration_pin, GPIO.OUT)
#GPIO.setup(realy_pin, GPIO.OUT)
#GPIO.setup(5, GPIO.OUT)
#GPIO.setup(6, GPIO.OUT)
#GPIO.setup(13, GPIO.OUT)
#GPIO.setup(19, GPIO.OUT)
buzzer = OutputDevice(18)
vibration = OutputDevice(27)
realy = OutputDevice(21)
#pin_A = OutputDevice(5)
#pin_B = OutputDevice(6)
#pin_C = OutputDevice(13)
#pin_D = OutputDevice(19)

pi.g_exit = True

#lcd_screen = LCDModule()
    
#RGB_close()
LedMatrix_close()
seven_close()
#initialize MCP
i2cAddr = 0x21  # MCP23008/17 i2c address
MCP = MCP230XX.MCP230XX('MCP23008', i2cAddr)
# Turn backlight on
blPin = 7 # 
MCP.set_mode(blPin, 'output')
lcd = HD44780MCP.HD44780(MCP, 1, -1, 2, [3,4,5,6], rows = 2, characters = 16, mode = 0, font = 0)
lcd.display(False)  
MCP.output(blPin, False)
#lcd_screen.clear()
#lcd_screen.turn_off()
#GPIO.output(buzzer_pin, False)
#GPIO.output(realy_pin, False)
#GPIO.output(vibration_pin, False)
#GPIO.output(5, False)
#GPIO.output(6, False)
#GPIO.output(13, False)
#GPIO.output(19, False)
#GPIO.cleanup()
buzzer.off()
vibration.off()
realy.off()
#pin_A.off()
#pin_B.off()
#pin_C.off()
#pin_D.off()