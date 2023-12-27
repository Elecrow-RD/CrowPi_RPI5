#!coding=UTF-8

import CrowpiAllModule as pi
import threading
import time
import atexit
import os
import sys

filePath = sys.argv[0]
fileFolderPath = filePath.replace("/main.py","/")
print(fileFolderPath)
#
#   程序退出时自动执行
#
@atexit.register 
def exit():
    global fileFolderPath
    os_command_0 = "sudo bash " + fileFolderPath + "kill.sh"
    print(os_command_0)
    os.system(os_command_0)
    print("所有模块已经退出!")
    
try:
    if __name__ == "__main__":

        #os_command_1 = "sudo python3 " + fileFolderPath + "ageTest_noPrint.py &"
        #os_command_2 = "sudo python3 " + fileFolderPath + "ageTest_manualPrint.py &"
        #os_command_3 = "sudo python3 " + fileFolderPath + "ageTest_autoPrint.py &"
        
        
        os_command_1 = "sudo python3 " + "/home/pi/Desktop/CrowPi/Drivers/aging-test/" + "ageTest_ultra.py &"
        os_command_2 = "sudo python3 " + "/home/pi/Desktop/CrowPi/Drivers/aging-test/" + "ageTest_light.py &"
        os_command_3 = "sudo python3 " + "/home/pi/Desktop/CrowPi/Drivers/aging-test/" + "ageTest_DH11.py &"
        os.system(os_command_1)
        os.system(os_command_2)
        os.system(os_command_3)
#        print("111")
#        time.sleep(1)
#        print("2222")
#        time.sleep(1)
#        time.sleep(20)


except KeyboardInterrupt:
    pi.g_exit = True
