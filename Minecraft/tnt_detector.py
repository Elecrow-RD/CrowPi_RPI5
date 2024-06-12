from gpiozero import Buzzer
import time
from mcpi.minecraft import Minecraft

mc = Minecraft.create() # create Minecraft Object

buzzer = Buzzer(18)

# repeat indefinitely
while True:
    # get player position
    x, y, z = mc.player.getPos()
    # look at every block until block 15
    for i in range(15):
        if mc.getBlock(x, y - i, z) == 46:
            buzzer.on() # buzz the buzzer on
            time.sleep(0.5) # wait
            buzzer.off() # turn the buzzer off
            time.sleep(0.5) # wait
