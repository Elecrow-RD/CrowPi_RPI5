#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
from gpiozero import OutputDevice as stepper

class StepMotor:
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
            


print("moving started")
motor = StepMotor()
while True:
    motor.ileri(stepSize=20)
    time.sleep(0.1)
    #motor.geri(stepSize=20)
    #time.sleep(0.1)

