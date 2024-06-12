#!/usr/bin/python
# -*- coding: utf-8 -*-
# Author : Original author ludwigschuster
# Original Author Github: https://github.com/ludwigschuster/RasPi-GPIO-Stepmotor
# http://elecrow.com/
from gpiozero import OutputDevice
import time
import math


# Declare pins as output
pin_A = OutputDevice(5)
pin_B = OutputDevice(6)
pin_C = OutputDevice(13)
pin_D = OutputDevice(19)
        
class Stepmotor:

    def __init__(self):
        self.interval = 0.010

    def Step1(self):
        pin_D.on()
        time.sleep(self.interval)
        pin_D.off()

    def Step2(self):
        pin_D.on()
        pin_C.on()
        time.sleep(self.interval)
        pin_D.off()
        pin_C.off()

    def Step3(self):
        pin_C.on()
        time.sleep(self.interval)
        pin_C.off()

    def Step4(self):
        pin_B.on()
        pin_C.on()
        time.sleep(self.interval)
        pin_B.off()
        pin_C.off()

    def Step5(self):
        pin_B.on()
        time.sleep(self.interval)
        pin_B.off()

    def Step6(self):
        pin_A.on()
        pin_B.on()
        time.sleep(self.interval)
        pin_A.off()
        pin_B.off()

    def Step7(self):
        pin_A.on()
        time.sleep(self.interval)
        pin_A.off()

    def Step8(self):
        pin_A.on()
        pin_D.on()
        time.sleep(self.interval)
        pin_D.off()
        pin_A.off()

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

    def close(self):
        # cleanup the GPIO pin use
        pin_A.close()
        pin_B.close()
        pin_C.close()
        pin_D.close()

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

def main():

    print("moving started")
    motor = Stepmotor()
    print("One Step")
    motor.turnSteps(1)
    time.sleep(0.5)
    print("20 Steps")
    motor.turnSteps(20)
    time.sleep(0.5)
    print("quarter turn")
    motor.turnDegrees(90)
    print("moving stopped")
    motor.close()

if __name__ == "__main__":
    main()
