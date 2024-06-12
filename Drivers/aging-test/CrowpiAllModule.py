# -*- coding: utf-8 -*-
#!/usr/bin/env python


#RGB_Matrix         RGB点阵
#Servo              伺服电机
#Stepmotor          步进电机
#Light              光敏传感器
#buzzer             蜂鸣器
#remote             人体红外传感器
#dh11               温湿度传感器
#readRFID           NFC模块
#light              光照传感器
#distance           超声波模块
#sound              声音传感器
#motion             人体红外传感器
#relay              继电器
#lcd                LCD1602
#segment            数码管
#tilt               倾斜传感器
#touch              触摸传感器
#vibration          震动马达
#step               步进电机
#moveServo          伺服电机
#matrixButtons      矩阵按键
#joystick           摇感模块
#
#


import time
import sys
#import Adafruit_CharLCD as LCD
import MCP230XX
import HD44780MCP
#from pirc522 import RFID
from mfrc522 import SimpleMFRC522
#import RPi.GPIO as GPIO
# from gpio import *
from gpiozero import DigitalInputDevice as IR
from gpiozero import DistanceSensor
from gpiozero import DigitalInputDevice as Sound
from gpiozero import DigitalInputDevice as MOTION
from gpiozero import OutputDevice as Relay
from gpiozero import DigitalInputDevice as Tilt
from gpiozero import Servo
from gpiozero import OutputDevice
from gpiozero import Buzzer, LED
from gpiozero import DigitalInputDevice as Touch
from gpiozero import OutputDevice as Vibration
from gpiozero import Button
from gpiozero import OutputDevice as stepper
#import Adafruit_DHT
from DFRobot_DHT20 import DFRobot_DHT20
import smbus
import math
import spidev
import datetime
#from rpi_ws281x import PixelStrip, Color
#from Adafruit_LED_Backpack import SevenSegment
from ht16k33segment_python import HT16K33Segment
import threading
import re
from luma.led_matrix.device import max7219
from luma.core.interface.serial import spi, noop
from luma.core.render import canvas
from luma.core.virtual import viewport
from luma.core.legacy import text, show_message
from luma.core.legacy.font import proportional, CP437_FONT, TINY_FONT, SINCLAIR_FONT, LCD_FONT

g_exit = False 

# set GPIO BCM mode
#GPIO.setmode(GPIO.BCM)

# Find the right revision for bus driver
#if(GPIO.RPI_REVISION == 1):
#    bus = smbus.SMBus(0)
#else:
bus = smbus.SMBus(1)

# Define LCD column and row size for 16x2 LCD.
#lcd_columns = 16
#lcd_rows    = 2

# Initialize the LCD using the pins
#lcd = LCD.Adafruit_CharLCDBackpack(address=0x21)
'''
# Open SPI bus
spi = spidev.SpiDev()
spi.open(0,1)
spi.max_speed_hz=1000000

def ReadChannel(channel):
  adc = spi.xfer2([1,(8+channel)<<4,0])
  data = ((adc[1]&3) << 8) + adc[2]
  return data
'''




class IR_Remote():

    #def __init__(self):
        #ir = IR(20)
        # set GPIO BCM mode
        #GPIO.setmode(GPIO.BCM)
        
        # set pin
        #self.pin = 20
        
        # setup IR pin as input
        #GPIO.setup(self.pin,GPIO.IN,GPIO.PUD_UP)

    def exec_cmd(self, key_val):
        if(key_val==0x45):
            print("@__INF_test:CH-#")
        elif(key_val==0x46):
            print("@__INF_test:CH#")
        elif(key_val==0x47):
            print("@__INF_test:CH+#")
        elif(key_val==0x44):
            print("@__INF_test:PREV#")
        elif(key_val==0x40):
            print("@__INF_test:NEXT#")
        elif(key_val==0x43):
            print("@__INF_test:PL/PA#")
        elif(key_val==0x07):
            print("@__INF_test:VOL-#")
        elif(key_val==0x15):
            print("@__INF_test:VOL+#")
        elif(key_val==0x09):
            print("@__INF_test:EQ#")
        elif(key_val==0x16):
            print("@__INF_test:0#")
        elif(key_val==0x19):
            print("@__INF_test:100+#")
        elif(key_val==0x0d):
            print("@__INF_test:200+#")
        elif(key_val==0x0c):
            print("@__INF_test:1#")
        elif(key_val==0x18):
            print("@__INF_test:2#")
        elif(key_val==0x5e):
            print("@__INF_test:3#")
        elif(key_val==0x08):
            print("@__INF_test:4#")
        elif(key_val==0x1c):
            print("@__INF_test:5#")
        elif(key_val==0x5a):
            print("@__INF_test:6#")
        elif(key_val==0x42):
            print("@__INF_test:7#")
        elif(key_val==0x52):
            print("@__INF_test:8#")
        elif(key_val==0x4a):
            print("@__INF_test:9#")
        time.sleep(0.5)
        #print("@__INF_test: #")
        #cleanup()

'''
class ButtonMatrix():

    def __init__(self):

        GPIO.setmode(GPIO.BCM)

        # Define key channels
        self.key_channel = 4
        self.delay = 0.1

        self.adc_key_val = [30,90,160,230,280,330,400,470,530,590,650,720,780,840,890,960]
        self.key = -1
        self.oldkey = -1
        self.num_keys = 16

        self.indexes = {
            12:1,
            13:2,
            14:3,
            15:4,
            10:7,
            9:6,
            8:5,
            11:8,
            4:9,
            5:10,
            6:11,
            7:12,
            0:13,
            1:14,
            2:15,
            3:16
        }

    def ReadChannel(self,channel):
        # Function to read SPI data from MCP3008 chip
        # Channel must be an integer 0-7
        adc = spi.xfer2([1,(8+channel)<<4,0])
        data = ((adc[1]&3) << 8) + adc[2]
        return data

    def GetAdcValue(self):
        adc_key_value = self.ReadChannel(self.key_channel)
        return adc_key_value

    def GetKeyNum(self,adc_key_value):
        for num in range(0,16):
            if adc_key_value < self.adc_key_val[num]:
                return num
        if adc_key_value >= self.num_keys:
            num = -1
            return num

    def activateButton(self, btnIndex):
        # get the index from SPI
        btnIndex = int(btnIndex)
        # correct the index to better format
        btnIndex = self.indexes[btnIndex]
#        print("按键%s" % btnIndex)
        # prevent button presses too close together
        time.sleep(.3)
        return btnIndex


class RGB_Matrix:

    def __init__(self):

        # LED strip configuration:
        self.LED_COUNT = 64        # Number of LED pixels.
        self.LED_PIN = 12          # GPIO pin connected to the pixels (18 uses PWM!).
        self.LED_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800khz)
        self.LED_DMA = 10          # DMA channel to use for generating signal (try 10)
        self.LED_BRIGHTNESS = 10  # Set to 0 for darkest and 255 for brightest
        self.LED_INVERT = False    # True to invert the signal (when using NPN transistor level shift)
        self.LED_CHANNEL = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53

        self.RIGHT_BORDER = [7,15,23,31,39,47,55,63]
        self.LEFT_BORDER = [0,8,16,24,32,40,48,56]

    # Define functions which animate LEDs in various ways.
    def clean(self,strip):
        # wipe all the LED's at once
        for i in range(strip.numPixels()):
            strip.setPixelColor(i, Color(0, 0, 0))
        strip.show()

    def rainbow(self,strip, wait_ms=20, iterations=1):
        """Draw rainbow that fades across all pixels at once."""
        global g_exit
        while g_exit is not True:
            for i in range(strip.numPixels()):
                strip.setPixelColor(i,Color(255,0,0))
            strip.show()
            time.sleep(0.5)
            for i in range(strip.numPixels()):
                strip.setPixelColor(i,Color(0,255,0))
            strip.show()
            time.sleep(0.5)
            for i in range(strip.numPixels()):
                strip.setPixelColor(i,Color(0,0,255))
            strip.show()
            time.sleep(0.5)
            for i in range(strip.numPixels()):
                strip.setPixelColor(i,Color(0,255,255))
            strip.show()
            time.sleep(0.5)
            for i in range(strip.numPixels()):
                strip.setPixelColor(i,Color(255,255,0))
            strip.show()
            time.sleep(0.5)
            for i in range(strip.numPixels()):
                strip.setPixelColor(i,Color(255,0,255))
            strip.show()
            time.sleep(0.5)
            for i in range(strip.numPixels()):
                strip.setPixelColor(i,Color(255,255,255))
            strip.show()
            time.sleep(0.5)

    def demo(self,strip):

        self.rainbow(strip)
        self.clean(strip)

    def run(self):
         # Create NeoPixel object with appropriate configuration.
         strip = PixelStrip(self.LED_COUNT, self.LED_PIN, self.LED_FREQ_HZ, self.LED_DMA, self.LED_INVERT, self.LED_BRIGHTNESS, self.LED_CHANNEL)
         # Intialize the library (must be called once before other functions).
         strip.begin()
         # do stuff
         try:
             #print('矩阵LED亮起')
             self.demo(strip)
         except KeyboardInterrupt:
             # clean the matrix LED before interruption
             self.clean(strip)
'''             


#class Servo:

   # def __init__( self, pin, direction ):
        # set GPIO BCM mode
        #GPIO.setmode(GPIO.BCM)
        #servo = Servo(25)
        #GPIO.setup( pin, GPIO.OUT )
        #self.pin = int( pin )
        #self.direction = int( direction )
        #self.servo = GPIO.PWM( self.pin, 50 )
        #self.servo.start(0.0)

    #def cleanup( self ):
        #self.servo.ChangeDutyCycle(self._henkan(0))
        #time.sleep(0.3)
        #self.servo.stop()
        #GPIO.cleanup()

    #def currentdirection( self ):
        #return self.direction

    #def _henkan( self, value ):
        #return 0.05 * value + 7.0

    #def setdirection( self, direction, speed ):
        #for d in range( self.direction, direction, int(speed) ):
            #self.servo.ChangeDutyCycle( self._henkan( d ) )
            #self.direction = d
            #time.sleep(0.3)
            #self.servo.ChangeDutyCycle( self._henkan( direction ) )
            #self.direction = direction

class Stepmotor:
    def __init__(self, mode=1, pins=[5,6,13,19]):
        self.mode = mode
        self.pins=pins
        self.IN1 = stepper(pins[0])
        self.IN2 = stepper(pins[1])
        self.IN3 = stepper(pins[2])
        self.IN4 = stepper(pins[3])   
        self.stepPins = [self.IN1,self.IN2,self.IN3,self.IN4]
        if self.mode:              # Low Speed ==> High Power
            self.seq = [[1,0,0,1], # Define step sequence as shown in manufacturers datasheet
                     [1,0,0,0], 
                     [1,1,0,0],
                     [0,1,0,0],
                     [0,1,1,0],
                     [0,0,1,0],
                     [0,0,1,1],
                     [0,0,0,1]]
        else:                      # High Speed ==> Low Power 
            self.seq = [[1,0,0,0], # Define step sequence as shown in manufacturers datasheet
                     [0,1,0,0],
                     [0,0,1,0],
                     [0,0,0,1]]

        self.stepCount = len(self.seq)

    def ileri(self, wait=0.01,stepSize=10):
        stepCounter = 0
        for dev in range(stepSize):                        
            for pin in range(0,4):
                xPin=self.stepPins[pin]          
                if self.seq[stepCounter][pin]!=0:
                    xPin.on()
                else:
                    xPin.off()
            
            stepCounter += 1
            if (stepCounter >= self.stepCount):
                stepCounter = 0
            time.sleep(wait)

    def geri(self, wait=0.01, stepSize=10):
        stepCounter = self.stepCount-1
        for dev in range(stepSize):
            for pin in range(0,4):
                xPin=self.stepPins[pin]          
                if self.seq[stepCounter][pin]!=0:
                    xPin.on()
                else:
                    xPin.off()
        
            stepCounter -= 1
            if (stepCounter == 0):
                stepCounter = self.stepCount-1
            time.sleep(wait)     
    def closepins(self):
        for i in range(0,4):
            self.stepPins[i].close()

class Light():
    def __init__(self):
        # Define some constants from the datasheet
        self.DEVICE = 0x5c # Default device I2C address

        self.POWER_DOWN = 0x00 # No active state
        self.POWER_ON = 0x01 # Power on
        self.RESET = 0x07 # Reset data register value

        # Start measurement at 4lx resolution. Time typically 16ms.
        self.CONTINUOUS_LOW_RES_MODE = 0x13
        # Start measurement at 1lx resolution. Time typically 120ms
        self.CONTINUOUS_HIGH_RES_MODE_1 = 0x10
        # Start measurement at 0.5lx resolution. Time typically 120ms
        self.CONTINUOUS_HIGH_RES_MODE_2 = 0x11
        # Start measurement at 1lx resolution. Time typically 120ms
        # Device is automatically set to Power Down after measurement.
        self.ONE_TIME_HIGH_RES_MODE_1 = 0x20
        # Start measurement at 0.5lx resolution. Time typically 120ms
        # Device is automatically set to Power Down after measurement.
        self.ONE_TIME_HIGH_RES_MODE_2 = 0x21
        # Start measurement at 1lx resolution.
        self.ONE_TIME_LOW_RES_MODE = 0x23

    def convertToNumber(self, data):
        # Simple function to convert 2 bytes of data
        # into a decimal number
        return ((data[1] + (256 * data[0])) / 1.2)

    def readLight(self):
        data = bus.read_i2c_block_data(self.DEVICE,self.ONE_TIME_HIGH_RES_MODE_1)
        return self.convertToNumber(data)

#def cleanup():
    #GPIO.cleanup()

def buzzer():
    global g_exit
    # set GPIO BCM mode
    buzzer = Buzzer(18)
    while g_exit is not True:
        # Make buzzer sound
        buzzer.on()
        time.sleep(0.5)
        buzzer.off()
        time.sleep(5)
        
    # Stop buzzer sound
    buzzer.off()

def remote():
    global g_exit
    ir = IR(20)
    while g_exit is not True:
        remote_object = IR_Remote()
        if ir.value == 0:
            count = 0
            while ir.value == 0 and count < 200:
                count += 1
                time.sleep(0.00006)
            
            count = 0
            while ir.value == 1 and count < 80:
                count += 1
                time.sleep(0.00006)
            idx = 0
            cnt = 0
            data = [0,0,0,0]

            for i in range(0,32):
                count = 0
                while ir.value == 0 and count < 15:
                    count += 1
                    time.sleep(0.00006)

                count = 0
                while ir.value == 1 and count < 40:
                    count += 1
                    time.sleep(0.00006)

                if count > 8:
                    data[idx] |= 1<<cnt
                if cnt == 7:
                    cnt = 0
                    idx += 1
                else:
                    cnt += 1

            if data[0]+data[1] == 0xFF and data[2]+data[3] == 0xFF:
                    #print("按键: 0x%02x" %data[2])
                    remote_object.exec_cmd(data[2])
                    #i = i + 1

def dh11():
    I2C_BUS     = 0x01  # default use I2C1 bus
    I2C_ADDRESS = 0x38  # default I2C device address
    dht20 = DFRobot_DHT20(I2C_BUS ,I2C_ADDRESS)
    global g_exit
    while g_exit is not True:
        temperature, humidity, crc_error = dht20.get_temperature_and_humidity()
        temperature =round(temperature,2)
        humidity = round(humidity,2)
        if humidity is not None and temperature is not None:
            valueStr = "@_DH11_test:T {0}*C   H {1}%".format(temperature, humidity) + "#"
            print(valueStr)
            sys.stdout.flush()
        else:
            print('error:dh11!')
            sys.stdout.flush()
#        time.sleep()

def readRFID():
    global g_exit
    reader = SimpleMFRC522()
    # set a flag
    while g_exit is not True:
        if(g_exit == True):
            break
       
        id, text = reader.read()
        valueStr = "@_RFID_test:成功检测#"
        print(valueStr)
        time.sleep(3)
        print("@_RFID_test: #")
        '''
        print(id)
        print(text)
        (error, data) = rdr.request()
        if not error:
            valueStr = "@_RFID_test:成功检测#"
            print(valueStr)
        (error, uid) = rdr.anticoll()
        if not error:
            time.sleep(1)
            print("@_RFID_test: #")
        '''

def light():
    global g_exit
    sensor = Light()
    while g_exit is not True:
        valueStr =  "@Light_test:" + str(int(sensor.readLight())) + " lx#"
        print(valueStr)
        time.sleep(0.5) 

def Ultrasonic():

    # set GPIO BCM mode
    #GPIO.setmode(GPIO.BCM)

    #TRIG = 16
    #ECHO = 12

    #GPIO.setup(TRIG,GPIO.OUT)
    #GPIO.setup(ECHO,GPIO.IN)

    #GPIO.output(TRIG, False)
    #time.sleep(2)

    #GPIO.output(TRIG, True)
    #time.sleep(0.00001)
    #GPIO.output(TRIG, False)

    #while GPIO.input(ECHO)==0:
     # pulse_start = time.time()

    #while GPIO.input(ECHO)==1:
     # pulse_end = time.time()

    #pulse_duration = pulse_end - pulse_start

    #distance = pulse_duration * 17150

    #distance = round(distance, 2)
    distancesensor = DistanceSensor(echo = 12,trigger = 16, max_distance = 5)
    distance = distancesensor.distance * 100
    distance = round(distance,2)
    valueStr =  "@Ultra_test:" + str(distance) + " cm#"
    print(valueStr)
#    return distance

def distance():
    global g_exit
    while g_exit is not True:
        dis=Ultrasonic()
        #time.sleep(0.5)

def sound():
    # set GPIO BCM mode
    #GPIO.setmode(GPIO.BCM)
    # define sound pin
    #sound_pin = 24
    # setup pin as INPUT
    #GPIO.setup(sound_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    sound_sensor = Sound(24)
    global g_exit
    while g_exit is not True:
        print("@Sound_test: #")
        # check if sound detected or not
        while(sound_sensor.value == 0):
            i = 1
        print('@Sound_test:成功检测#')
        time.sleep(0.5)


def motion():

    # set GPIO BCM mode
   # GPIO.setmode(GPIO.BCM)
    # define motion pin
    #motion_pin = 23
    motion = MOTION(23)
    # set pin mode as INPUT
    #GPIO.setup(motion_pin, GPIO.IN,pull_up_down=GPIO.PUD_UP)
    moves = False
    unmoves = False
    global g_exit
    while g_exit is not True:
        if(motion.value == 0):
            print("@__PIR_test: #")
        elif(motion.value == 1):
            print("@__PIR_test:成功检测#")
        time.sleep(0.5)

def relay():

    # set GPIO BCM mode
    #GPIO.setmode(GPIO.BCM)
    # define relay pin
    relay = Relay(21)

    # setup relay pin as OUTPUT
    #GPIO.setup(relay_pin, GPIO.OUT)
    global g_exit
    while g_exit is not True:
        # Open Relay
        #GPIO.output(relay_pin, GPIO.LOW)
        relay.off()
        # Wait half a second
        time.sleep(2)
        # Close Relay
        #GPIO.output(relay_pin, GPIO.HIGH)
        relay.on()
        time.sleep(2)
    # Close Relay
    #GPIO.output(relay_pin, GPIO.LOW)
    relay.off()

def lcd():
    global g_exit
    #initialize MCP
    i2cAddr = 0x21  # MCP23008/17 i2c address
    MCP = MCP230XX.MCP230XX('MCP23008', i2cAddr)
    # Turn backlight on
    blPin = 7 # 
    MCP.set_mode(blPin, 'output')
    MCP.output(blPin, True) 
    lcd = HD44780MCP.HD44780(MCP, 1, -1, 2, [3,4,5,6], rows = 2, characters = 16, mode = 0, font = 0)
    
    while g_exit is not True:
        lcd.display_string('RRRRRRRRRRRRRRRR')
        lcd.set_cursor(2,1)
        lcd.display_string('RRRRRRRRRRRRRRRR')
        time.sleep(2)

    lcd.display(False)  
    MCP.output(blPin, False)

def Led_matrix():
    global g_exit
    # create matrix device
    serial = spi(port=0, device=1, gpio=noop())
    device = max7219(serial, cascaded=1, block_orientation=90, rotate=0)

    # print hello world on the matrix display
    msg = "123456789"
    while g_exit is not True:
        # debugging purpose
        show_message(device, msg, fill="white", font=proportional(CP437_FONT), scroll_delay=0.1)
        time.sleep(1)
        #GPIO.cleanup()

def segment():
    i2c = smbus.SMBus(1)
    segment = HT16K33Segment(i2c)
    segment.set_brightness(15)
  
    global g_exit
    while g_exit is not True:
        for i in range (0,10):
            if g_exit is True:
                break
            segment.clear()
            segment.set_number(i, 0,  True)     
            segment.set_number(i, 1,  True)          
            segment.set_number(i, 2,  True)   
            segment.set_number(i, 3,  True)       
            segment.set_colon(True)              
            segment.update()
            time.sleep(0.5)
#        segment.set_colon(0)              # Toggle colon at 1Hz
        segment.clear()
        segment.update()
        time.sleep(0.5)

def tilt():

    # set GPIO BCM mode
    #GPIO.setmode(GPIO.BCM)
    # define tilt pin
    #tilt_pin = 22
    # set puin as input
    #GPIO.setup(tilt_pin, GPIO.IN,pull_up_down=GPIO.PUD_UP)
    tilt_sensor = Tilt(22)
    tilt_left = False
    tilt_right = False

    global g_exit
    while g_exit is not True:
        # positive is tilt to left negative is tilt to right
        if tilt_sensor.value:
            print("@_Tilt_test: 左倾斜#")
        else:
            print("@_Tilt_test: 右倾斜#")
        time.sleep(0.5)

def touch():

    # set GPIO BCM mode
    #GPIO.setmode(GPIO.BCM)
    touch = False
    #touch_pin = 17
    touch_sensor = Touch(17)
    # set GPIO pin to INPUT
    #GPIO.setup(touch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    global g_exit
    while g_exit is not True:
        # check if touch detected
        if touch_sensor.value:
            print('@Touch_test:成功检测#')
        else:
            print("@Touch_test: #")
        time.sleep(0.5)

def vibration():

    # set GPIO BCM mode
    #GPIO.setmode(GPIO.BCM)
    # define vibration pin
    vibration_sensor = Vibration(27)

    global g_exit
    while g_exit is not True:
        # turn on vibration
        vibration_sensor.on()
        time.sleep(0.5)
        vibration_sensor.off()
        time.sleep(5)
    # turn off vibration
    vibration_sensor.off()

def step():
    motor = Stepmotor()
    global g_exit
    while g_exit is not True:
        #print("开始转动")
        motor.ileri(stepSize=20)
        time.sleep(0.1)

def servo():
    global g_exit
    servo2 = Servo(25)
    while g_exit is not True:
        servo2.min()
        time.sleep(1)
        servo2.mid()
        time.sleep(1)
        servo2.max()
        time.sleep(1)
    #s.cleanup()

'''
def matrixButtons():

    # initial the button matrix
    buttons = ButtonMatrix()
    #while True:
    global g_exit
    while g_exit is not True:
        # get buttons press from SPI
        adc_key_value = buttons.GetAdcValue()
        key = buttons.GetKeyNum(adc_key_value)
        if key != buttons.oldkey:
            time.sleep(0.05)
            adc_key_value = buttons.GetAdcValue()
            key = buttons.GetKeyNum(adc_key_value)
            if key != buttons.oldkey:
                oldkey = key
                if key >= 0:
                    # button pressed, activate it
                    btnIndex = buttons.activateButton(key)
                    valueStr = "@__Btn_test:" + str(btnIndex) + "#"
                    print(valueStr)
                    time.sleep(0.5)
                    print("@__Btn_test: #")

        

def joystick():

    # Define key channels
    x_channel = 1
    y_channel = 0

    delay = 0.15

    # create empty counter
    try:
        global g_exit
        while g_exit is not True:
                            
            # Read the  data
            x_value = ReadChannel(x_channel)
            y_value = ReadChannel(y_channel)
            if x_value > 650:
                print("@Joyst_test:左#")
                #l += 1
            elif x_value < 400:
                print("@Joyst_test:右#")
                #r += 1
            elif y_value > 650:
                print("@Joyst_test:上#")
                #u += 1
            elif y_value < 400:
                print("@Joyst_test:下#")
            else:
                print("@Joyst_test: #")
                #d += 1
            time.sleep(0.5)
 
    except KeyboardInterrupt:
        GPIO.cleanup()
'''

def main():
    print("****************************************************************")
    print("[-] CrowPi2 硬件模块测试程序 ...")
    print("[-] 每测试一个模块，如没问题则按下Enter键以继续测试下一个模块！")
    #time.sleep(1)
    print("\n\n[-] 请将IR接收器插入CrowPi2以测试IR模块...")
    print("[-] 请按下遥控器上任意按键，CrowPi2应打印按键信息")
    #print("[-] 按下Enter键确认模块没问题，测试下一模块！")
    #time.sleep(1)
    remote()
    cleanup()
    print("[-] IR模块测试通过！")
    time.sleep(1)
    
    print("\n\n[-] 温湿度传感器 DHT11 测试...")
    print("[-] CrowPi2应打印出温度和湿度信息")
    #print("[-] 按下Enter键确认模块没问题，测试下一模块！")
    dh11()
    cleanup()
    print("[-] 温湿度传感器 DHT11测试通过！")
    time.sleep(1)
    
    #print("\n\n[-] RFID模块测试...")
    #print("[-] 将NFC卡放在RFID模块区域，CrowPi2应打印'成功读取到NFC卡'")
    #print("[-] 按下Enter键确认模块没问题，测试下一模块！")
    #readRFID()
    #cleanup()
    #print("[-] RFID模块测试通过！")
    #time.sleep(1)
    
    print("\n\n[-] 触摸传感器测试...")
    print("[-] 触摸传感器感应区，应打印出'成功检测触摸'")
    #print("[-] 按下Enter键确认模块没问题，测试下一模块！")
    touch()
    cleanup()
    print("[-] 触摸传感器测试通过！")
    time.sleep(1)
    
    print("\n\n[-] 距离传感器测试...")
    print("[-] 将手放在超声波上方并改变距离，应打印出超声波检测到与手的距离")
    #print("[-] 按下Enter键确认模块没问题，测试下一模块！")
    distance()
    cleanup()
    print("[-] 距离传感器测试通过！")
    time.sleep(1)
    
    print("\n\n[-] 声音传感器测试...")
    print("[-] 对着声音传感器吼一下，应打印出'成功检测到声音'，如太灵敏或不灵敏请调节声音传感器的灵敏度")
    #print("[-] 按下Enter键确认模块没问题，测试下一模块！")
    sound()
    cleanup()
    print("[-] 声音传感器测试通过！")
    time.sleep(1)
    
    print("\n\n[-] 运动传感器测试...")
    print("[-] 在运动传感器上方20-30厘米处挥手，应打印出'成功检测到运动'，如太灵敏或不灵敏请调节运动传感器的灵敏度")
    #print("[-] 按下Enter键确认模块没问题，测试下一模块！")
    motion()
    cleanup()
    print("[-] 运动传感器测试通过！")
    time.sleep(1)
    
    print("\n\n[-] 继电器测试...")
    print("[-] 继电器应发出不断开关的滴答声")
    #print("[-] 按下Enter键确认模块没问题，测试下一模块！")
    relay()
    cleanup()
    print("[-] 继电器测试通过！")
    time.sleep(1)
    
    print("\n\n[-] 倾斜传感器测试...")
    print("[-] 一开始应打印'向左倾斜'或'向右倾斜'，你应往相反的另一侧倾斜且CrowPi2应打印出来")
    #print("[-] 按下Enter键确认模块没问题，测试下一模块！")
    tilt()
    cleanup()
    print("[-] 倾斜传感器测试通过！")
    time.sleep(1)
    
    
    try:
        '''
        print("\n\n[-] 遥感模块测试...")
        print("[-] 向左右上下四个方向摇动摇杆，应打印出摇杆的转向")
       # print("[-] 按下Enter键确认模块没问题，测试下一模块！")
        joystick()
        cleanup()
        print("[-] 遥感模块测试通过！")
        time.sleep(1)
        
        print("\n\n[-] 矩阵按键测试...")
        print("[-] 依次按下16个按键，每次按下将打印出被按下按键信息")
        #print("[-] 按下Enter键确认模块没问题，测试下一模块！")
        matrixButtons()
        cleanup()
        print("[-] 矩阵按键测试通过！")
        '''    
    except KeyboardInterrupt:
        cleanup()

try:
    if __name__ == "__main__":

        #t1 = threading.Thread(target = activate)
        #t1.start()
        print("[-] 4字数码管、LCD、亮度传感器、矩阵LED、伺服电机、步进电机、蜂鸣器、振动马达测试...")
        print("[-] 数码管应亮起4个8，且中间的冒号应一会亮一会灭")
        print("[-] LCD背光应亮起，显示亮度传感器的数据")
        print("[-] 矩阵LED应在红、绿、蓝三种颜色不停变换")
        print("[-] 私服电机应在一直左,右转动")
        print("[-] 步进电机应一直转动")
        print("[-] 蜂鸣器应发出蜂鸣声")
        print("[-] 手摸振动马达应能感受到振动且马达振动发出的声音")
        t2 = threading.Thread(target = segment)
        t2.start()
        t3 = threading.Thread(target = lcd)
        t3.start()
        t4 = threading.Thread(target = step)
        t4.start()
        t5 = threading.Thread(target = servo)
        t5.start()
        Led_matrix()
        '''
        matrix = RGB_Matrix()
        while(act_key == False):
            matrix.run()
            time.sleep(0.25)
            #time.sleep()
        #raw = input("[!]请按下键盘回车键进行写一步测试!")
        #print("[-] 矩阵LED测试通过！")
        time.sleep(2)
        '''
        main()
        cleanup()
        print("\n硬件模块测试完成！请移除连接到CrowPi2的所有外接模块！")
        print("任一模块有问题，请务必标注并反馈！！！")
        print("****************************************************************")
        
except KeyboardInterrupt:
    cleanup()


