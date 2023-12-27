import random
from mcpi.minecraft import Minecraft
from mcpi import block
import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522

tnt = 64
reader = SimpleMFRC522()
mc = Minecraft.create()
id, text = reader.read()
print(id)
print(text)

if int(text) == tnt:
    print("set tnt")
    x, y, z = mc.player.getPos()
    for i in range(20):
        j = random.randint(0,20)
        k = random.randint(0,20)
        mc.setBlock(x + j,y,z + k,block.TNT.id)

