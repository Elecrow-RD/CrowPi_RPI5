from mcpi.minecraft import Minecraft
from gpiozero import Button
import time

# store the GPIO control pins
UP_PIN    =  Button(26)
DOWN_PIN  =  Button(13)
LEFT_PIN  =  Button(25)
RIGHT_PIN =  Button(19)


# create Minecraft Object
mc = Minecraft.create()

while True:
    x,y,z = mc.player.getPos()
    if UP_PIN.is_pressed:
        mc.player.setPos(x-0.1, y, z+0.1)
        print("Moving up ...")
    if DOWN_PIN.is_pressed:
        mc.player.setPos(x+0.1, y, z-0.1)
        print("Moving down ...")
    if LEFT_PIN.is_pressed:
        print("Moving left ...")
    if RIGHT_PIN.is_pressed:
        print("Moving right ...")
