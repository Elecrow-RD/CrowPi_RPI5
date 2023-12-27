# -*- coding: utf-8 -*-
#!/usr/bin/env python

import time
import sys
import socket, signal
from array import array
import re
import HD44780MCP
import MCP230XX
from mfrc522 import SimpleMFRC522
import RPi.GPIO as GPIO
from DFRobot_DHT20 import DFRobot_DHT20
import smbus
import math
import datetime
from ht16k33segment_python import HT16K33Segment
from luma.led_matrix.device import max7219
from luma.core.interface.serial import spi, noop
from luma.core.render import canvas
from luma.core.virtual import viewport
from luma.core.legacy import text, show_message
from luma.core.legacy.font import proportional, CP437_FONT, TINY_FONT, SINCLAIR_FONT, LCD_FONT

# set GPIO BCM mode
GPIO.setmode(GPIO.BCM)

# Find the right revision for bus driver
if(GPIO.RPI_REVISION == 1):
    bus = smbus.SMBus(0)
else:
    bus = smbus.SMBus(1)

class ButtonMatrix():

    def __init__(self):

        # set GPIO BCM mode
        GPIO.setmode(GPIO.BCM)

        # matrix button ids
        self.buttonIDs = [[1,2,3,4],[5,6,7,8],[9,10,11,12],[13,14,15,16]]
        # gpio inputs for rows
        self.rowPins = [27,22,5,6]
        # gpio outputs for columns
        self.columnPins = [13,19,26,25]

        # define four inputs with pull up resistor
        for i in range(len(self.rowPins)):
            GPIO.setup(self.rowPins[i], GPIO.IN, pull_up_down = GPIO.PUD_UP)

        # define four outputs and set to high
        for j in range(len(self.columnPins)):
            GPIO.setup(self.columnPins[j], GPIO.OUT)
            GPIO.output(self.columnPins[j], 1)

    def activateButton(self, rowPin, colPin):
        # get the button index
        btnIndex = self.buttonIDs[rowPin][colPin] - 1
        print("button " + str(btnIndex + 1) + " pressed")
        # prevent button presses too close together
        time.sleep(.3)

    def buttonHeldDown(self,pin):
        if(GPIO.input(self.rowPins[pin]) == 0):
            return True
            return False

class Servo:

    def __init__( self, pin, direction ):

        # set GPIO BCM mode
        GPIO.setmode(GPIO.BCM)

        GPIO.setup( pin, GPIO.OUT )
        self.pin = int( pin )
        self.direction = int( direction )
        self.servo = GPIO.PWM( self.pin, 50 )
        self.servo.start(0.0)

    def cleanup( self ):

        self.servo.ChangeDutyCycle(self._henkan(0))
        time.sleep(0.3)
        self.servo.stop()
        GPIO.cleanup()

    def currentdirection( self ):

        return self.direction

    def _henkan( self, value ):

        return 0.05 * value + 7.0

    def setdirection( self, direction, speed ):

        for d in range( self.direction, direction, int(speed) ):
            self.servo.ChangeDutyCycle( self._henkan( d ) )
            self.direction = d
            time.sleep(0.1)
            self.servo.ChangeDutyCycle( self._henkan( direction ) )
            self.direction = direction

class Stepmotor:

    def __init__(self):

        # set GPIO BCM mode
        GPIO.setmode(GPIO.BCM)

        # These are the pins which will be used on the Raspberry Pi
        self.pin_A = 5
        self.pin_B = 6
        self.pin_C = 13
        self.pin_D = 19
        self.interval = 0.010

        # Declare pins as output
        GPIO.setup(self.pin_A,GPIO.OUT)
        GPIO.setup(self.pin_B,GPIO.OUT)
        GPIO.setup(self.pin_C,GPIO.OUT)
        GPIO.setup(self.pin_D,GPIO.OUT)
        GPIO.output(self.pin_A, False)
        GPIO.output(self.pin_B, False)
        GPIO.output(self.pin_C, False)
        GPIO.output(self.pin_D, False)

    def Step1(self):

        GPIO.output(self.pin_D, True)
        time.sleep(self.interval)
        GPIO.output(self.pin_D, False)

    def Step2(self):

        GPIO.output(self.pin_D, True)
        GPIO.output(self.pin_C, True)
        time.sleep(self.interval)
        GPIO.output(self.pin_D, False)
        GPIO.output(self.pin_C, False)

    def Step3(self):

        GPIO.output(self.pin_C, True)
        time.sleep(self.interval)
        GPIO.output(self.pin_C, False)

    def Step4(self):

        GPIO.output(self.pin_B, True)
        GPIO.output(self.pin_C, True)
        time.sleep(self.interval)
        GPIO.output(self.pin_B, False)
        GPIO.output(self.pin_C, False)

    def Step5(self):

        GPIO.output(self.pin_B, True)
        time.sleep(self.interval)
        GPIO.output(self.pin_B, False)

    def Step6(self):

        GPIO.output(self.pin_A, True)
        GPIO.output(self.pin_B, True)
        time.sleep(self.interval)
        GPIO.output(self.pin_A, False)
        GPIO.output(self.pin_B, False)

    def Step7(self):

        GPIO.output(self.pin_A, True)
        time.sleep(self.interval)
        GPIO.output(self.pin_A, False)

    def Step8(self):

        GPIO.output(self.pin_D, True)
        GPIO.output(self.pin_A, True)
        time.sleep(self.interval)
        GPIO.output(self.pin_D, False)
        GPIO.output(self.pin_A, False)

    def turn(self,count):
        for i in range (int(count)):
            self.Step1()
            self.Step2()
            self.Step3()
            self.Step4()
            self.Step5()
            self.Step6()
            self.Step7()
            self.Step8()

    def turnSteps(self, count):
        # Turn n steps
        # (supply with number of steps to turn)
        for i in range (count):
            self.turn(1)

    def turnDegrees(self, count):
        # Turn n degrees (small values can lead to inaccuracy)
        # (supply with degrees to turn)
        self.turn(round(count*512/360,0))

    def turnDistance(self, dist, rad):
        # Turn for translation of wheels or coil (inaccuracies involved e.g. due to thickness of rope)
        # (supply with distance to move and radius in same metric)
        self.turn(round(512*dist/(2*math.pi*rad),0))

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

def cleanup():

    GPIO.cleanup()

def buzzer():

    # set GPIO BCM mode
    GPIO.setmode(GPIO.BCM)

    buzzer_pin = 18

    GPIO.setup(buzzer_pin, GPIO.OUT)
    # Make buzzer sound
    GPIO.output(buzzer_pin, GPIO.HIGH)
    time.sleep(0.5)
    # Stop buzzer sound
    GPIO.output(buzzer_pin, GPIO.LOW)

def dht20():
    I2C_BUS     = 0x01  # default use I2C1 bus
    I2C_ADDRESS = 0x38  # default I2C device address
    dht20 = DFRobot_DHT20(I2C_BUS ,I2C_ADDRESS)
    temperature, humidity, crc_error = dht20.get_temperature_and_humidity()
    if crc_error:
      print("CRC               : Error\n")
    if humidity is not None and temperature is not None:
        print('Temp={0:0.1f}*  Humidity={1:0.1f}%'.format(temperature, humidity))
    else:
        print('Failed to get reading. Try again!')



def readRFID():
    reader = SimpleMFRC522()
    # set a flag
    continue_reading = True
    # print instructions
    print("[-] Touch with your card over the RC522 to read NFC data")
    while continue_reading:
        id, text = reader.read()
        print(id)
        print(text)
        continue_reading = False
        

def light():

    sensor = Light()
    while True:
        print("Light Level : " + str(sensor.readLight()) + " lx")
        time.sleep(0.5)
        if(sensor.readLight()<50):
            break

def distance():

    # set GPIO BCM mode
    GPIO.setmode(GPIO.BCM)

    TRIG = 16
    ECHO = 12

    GPIO.setup(TRIG,GPIO.OUT)
    GPIO.setup(ECHO,GPIO.IN)

    GPIO.output(TRIG, False)
    time.sleep(2)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO)==0:
      pulse_start = time.time()

    while GPIO.input(ECHO)==1:
      pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    distance = pulse_duration * 17150

    distance = round(distance, 2)

    print("[-] Distance: ",distance,"cm")
    return distance

def dis_test():
    while True:
        dis=distance()
        if(dis<50):
            break

def matrix(cascaded=1, block_orientation=90, rotate=0):

    # create matrix device
    serial = spi(port=0, device=1, gpio=noop())
    device = max7219(serial, cascaded=cascaded or 1, block_orientation=block_orientation, rotate=rotate or 0)

    # print hello world on the matrix display
    msg = "123456"
    # debugging purpose
    show_message(device, msg, fill="white", font=proportional(CP437_FONT), scroll_delay=0.1)

def sound():

    # set GPIO BCM mode
    GPIO.setmode(GPIO.BCM)
    # define sound pin
    sound_pin = 24
    # setup pin as INPUT
    GPIO.setup(sound_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    while True:
        # check if sound detected or not
        if(GPIO.input(sound_pin)==GPIO.LOW):
            print('Sound Detected')
            time.sleep(1)
            break

def motion():

    # set GPIO BCM mode
    GPIO.setmode(GPIO.BCM)
    # define motion pin
    motion_pin = 23

    # set pin mode as INPUT
    GPIO.setup(motion_pin, GPIO.IN)
    moves = False
    unmoves = False
    while (moves==False or unmoves==False):
        if(GPIO.input(motion_pin) == 0):
            print(" Nothing moves ...")
            unmoves==True
        elif(GPIO.input(motion_pin) == 1):
            print(" moves ...")
            moves=True
            break
        time.sleep(0.5)

def relay():

    # set GPIO BCM mode
    GPIO.setmode(GPIO.BCM)
    # define relay pin
    relay_pin = 21

    # setup relay pin as OUTPUT
    GPIO.setup(relay_pin, GPIO.OUT)

    # Open Relay
    GPIO.output(relay_pin, GPIO.LOW)
    # Wait half a second
    time.sleep(1.5)
    # Close Relay
    GPIO.output(relay_pin, GPIO.HIGH)
    time.sleep(1.5)
    # Close Relay
    GPIO.output(relay_pin, GPIO.LOW)

def lcd():
    #initialize MCP
    i2cAddr = 0x21  # MCP23008/17 i2c address
    MCP = MCP230XX.MCP230XX('MCP23008', i2cAddr)
    # Turn backlight on
    blPin = 7 # 
    MCP.set_mode(blPin, 'output')
    MCP.output(blPin, True) 
    lcd = HD44780MCP.HD44780(MCP, 1, -1, 2, [3,4,5,6], rows = 2, characters = 16, mode = 0, font = 0)
    lcd.display_string('Hello world!')
    time.sleep(10)
    print("[!]调试可调电阻，使其显示正常，按下回车键，程序继续运行！")
    lcd.display(False)  
    MCP.output(blPin, False)



def segment():
    i2c = smbus.SMBus(1)
    segment = HT16K33Segment(i2c)
    segment.set_brightness(15)

    for i in range (0,10):
        segment.clear()
        segment.set_number(i, 0,  True)     
        segment.set_number(i, 1,  True)          
        segment.set_number(i, 2,  True)   
        segment.set_number(i, 3,  True)       
        segment.set_colon(True)              
        segment.update()
        time.sleep(1)

    segment.clear()
    segment.update()


def tilt():

    # set GPIO BCM mode
    GPIO.setmode(GPIO.BCM)
    # define tilt pin
    tilt_pin = 22
    # set puin as input
    GPIO.setup(tilt_pin, GPIO.IN)

    tilt_left = False
    tilt_right = False

    while tilt_left == False or tilt_right == False:
        # positive is tilt to left negative is tilt to right
        if GPIO.input(tilt_pin):
            print("[-] Left Tilt")
            tilt_left = True
        else:
            print("[-] Right Tilt")
            tilt_right = True
        time.sleep(1)

def touch():

    # set GPIO BCM mode
    GPIO.setmode(GPIO.BCM)
    touch = False
    touch_pin = 17
    # set GPIO pin to INPUT
    GPIO.setup(touch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    while touch == False:
        # check if touch detected
        if(GPIO.input(touch_pin)):
            print('Touch Detected')
            touch = True
        else:
            print("No touch, please touch the touch sensor")
        time.sleep(0.5)

def vibration():

    # set GPIO BCM mode
    GPIO.setmode(GPIO.BCM)
    # define vibration pin
    vibration_pin = 27
    # Setup vibration pin to OUTPUT
    GPIO.setup(vibration_pin, GPIO.OUT)
    # turn on vibration
    GPIO.output(vibration_pin, GPIO.HIGH)
    # wait half a second
    time.sleep(1.5)
    # turn off vibration
    GPIO.output(vibration_pin, GPIO.LOW)

def step():

    print("moving started")
    motor = Stepmotor()
    print("quarter turn")
    motor.turnDegrees(90)
    print("moving stopped")

def moveServo():

    servo_pin = 25
    s = Servo(servo_pin,0)
    print("Turn left ...")
    s.setdirection( 100, 10 )
    time.sleep(0.5)
    print("Turn right ...")
    s.setdirection( -100, -10 )
    time.sleep(0.5)
    s.cleanup()

def indeButtons():

    # set GPIO BCM mode
    GPIO.setmode(GPIO.BCM)
    buttons = [26,13,19,25]

    buttons_clicked = []

    for button in buttons:
        GPIO.setup(button, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    while len(buttons) != len(buttons_clicked):
        for button in buttons:
            if(GPIO.input(button) == 0):
                print("%s GPIO button clicked" % button)
                buttons_clicked.append(button)
                time.sleep(0.5)

def matrixButtons():

     # initial the button matrix
     buttons = ButtonMatrix()
     while(True):
         for j in range(len(buttons.columnPins)):
             # set each output pin to low
             GPIO.output(buttons.columnPins[j],0)
             for i in range(len(buttons.rowPins)):
                 if GPIO.input(buttons.rowPins[i]) == 0:
                     # button pressed, activate it
                     buttons.activateButton(i,j)
                     # do nothing while button is being held down
                     while(buttons.buttonHeldDown(i)):
                         pass
              # return each output pin to high
             GPIO.output(buttons.columnPins[j],1)
def main():

    print("[-] CROWPI 测试程序 ...")
    time.sleep(1)
    print("[-] 蜂鸣器测试 如果听到一声蜂鸣器的叫声，蜂鸣器正常，如果没有听到蜂鸣器异常...")
    buzzer()
    cleanup()
    time.sleep(2)
    print("[-] 亮度传感器测试 ，需要用手挡住传感器部分光线，试亮度下降到50lx测试通过，如果不能到50lx测试无法通过...")
    light()
    cleanup()
    time.sleep(2)
    print("[-] 温湿度传感器 DH11 ，显示温度与湿度的数据，传感器正常...")
    dht20()
    cleanup()
    time.sleep(2)
    print("[-] NFC测试 ，等待复旦卡的刷卡动作，如果没有此动作会一直等待...")
    readRFID()
    cleanup()
    time.sleep(2)
    print("[-] 触摸传感器测试，等待触摸动作，如果没有动作，会一直扫描")
    touch()
    cleanup()
    time.sleep(2)
    print("[-] 距离传感器测试，需要用手挡住试距离在50cm以内，如果距离过长无法通过测试...")
    dis_test()
    cleanup()
    time.sleep(2)
    print("[-] 8*8led矩阵测试，正常显示就可以...")
    matrix()
    cleanup()
    time.sleep(2)
    print("[-] 声音传感器测试 ，需要发出一些声音，如果没有会一直等待...")
    sound()
    cleanup()
    time.sleep(2)
    print("[-] 运动传感器测试 ，等待运动动作，如果没有动作会一直等待...")
    motion()
    cleanup()
    time.sleep(2)
    print("[-] 继电器测试 ，观察继电器的指示灯是否亮起，并听到继电器打开的声音，如果没有，说明继电器有异常...")
    relay()
    cleanup()
    time.sleep(2)
    print("[-] LCD测试，观察lcd的显示，如果没有显示，可以调节可调电阻.....")
    lcd()
    cleanup()
    time.sleep(2)
    print("[-] 数码管测试，观察数码管是否正常显示....")
    segment()
    cleanup()
    time.sleep(2)
    print("[!]请打开UX5的8个开关，然后按下键盘回车键进行写一步测试!")
    print("[-] 倾斜传感器测试，默认是左边，需要将转向右边倾斜")
    tilt()
    cleanup()
    time.sleep(2)
    print("[-] 振动马达测试 ，是否听到马达开启的生音...")
    vibration()
    cleanup()
    time.sleep(2)
    print("[-] 步进电机测试 ，观察步进电机是否转动...")
    step()
    cleanup()
    time.sleep(2)
    print("[-] 伺服电机测试 ，观察私服电机是否转动...")
    moveServo()
    cleanup()
    time.sleep(2)
    print("[!] 将UX5的8个开关全部关闭，全部打开UX1的8个开关,按下回车键程序运行！")
    print("[-] 独立按键测试，请依次按下UP,DOWN,LEFT,RIGHT ...")
    indeButtons()
    cleanup()
    time.sleep(2)
    print("[-] 按键矩阵测试，请依次按下16个按键 ...")
    try:
        matrixButtons()
    except KeyboardInterrupt:
        cleanup()

if __name__ == "__main__":
    main()
