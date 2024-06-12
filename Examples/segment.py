from ht16k33segment_python import HT16K33Segment
import smbus
import time



def count(max, display, delay=1):
    if max < 0:
        max = 0
    if max > 9998:
        max = 9998
    for i in range(0, max + 1):
        bcd = int(str(i), 16)
        display.set_number((bcd & 0xF000) >> 12, 0)
        display.set_number((bcd & 0x0F00) >> 8, 1)
        display.set_number((bcd & 0xF0) >> 4, 2)
        display.set_number((bcd & 0x0F), 3)
        display.update()
        time.sleep(delay)


def anim(number, display, delay=1):
    state = True
    up = 99
    down = 92
    for i in range(0, number):
        display.clear()
        display.set_glyph(up if state else down, 0)
        display.set_glyph(down if state else up, 1)
        display.set_glyph(up if state else down, 2)
        display.set_glyph(down if state else up, 3)
        display.update()
        time.sleep(delay)
        state = not state


i2c = smbus.SMBus(1)
led = HT16K33Segment(i2c)
led.set_brightness(15)

count(1000, led, 0.01)
time.sleep(9)

led.clear()
led.set_number(8, 0, True)
led.set_number(8, 1, True)
led.set_number(8, 2, True)
led.set_number(8, 3, True)
led.set_colon(True)
led.update()
time.sleep(10)

led.clear()
led.set_char("b", 0)
led.set_char("e", 1)
led.set_char("e", 2)
led.set_char("f", 3)
led.update()

led.set_blink_rate(1)

time.sleep(10)

led.set_blink_rate(0)

time.sleep(10)

anim(20, led, 0.5)
time.sleep(8)

led.clear()
led.update()
