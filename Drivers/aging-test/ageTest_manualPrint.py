#!coding=UTF-8

import CrowpiAllModule as pi
import threading
import time
import atexit
import os

#
#   程序退出时自动执行
#
@atexit.register 
def exit():
    print("所有模块已经退出!")
    


#输出类
def outputTest():
    
    #摇杆模块
    #th_Joystick = threading.Thread(target = pi.joystick)
    #th_Joystick.start()
    
    #PIR人体红外传感器
    th_PIR = threading.Thread(target = pi.motion)
    th_PIR.start()

    #触摸传感器
    th_Touch = threading.Thread(target = pi.touch)
    th_Touch.start()
    
    #声音传感器
    th_Sound = threading.Thread(target = pi.sound)
    th_Sound.start()
    
    #8x8按键模块
    #th_Button8x8 = threading.Thread(target = pi.matrixButtons)
    #th_Button8x8.start()

def CloseAll():
    pi.g_exit = True
    print("正在关闭所有模块！")
    os.system("sudo bash kill.sh")
    



try:
    if __name__ == "__main__":
        outputTest()

except KeyboardInterrupt:
    pi.g_exit = True
