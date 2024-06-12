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
    
    
    #LCD1602显示
    th_LCD1602 = threading.Thread(target = pi.lcd)
    th_LCD1602.start()
    time.sleep(0.5)


    #数码管显示
    th_Segment = threading.Thread(target = pi.segment)
    th_Segment.start()
    time.sleep(0.5)
    
    
    #led点阵显示
    #te_LedMatrix = pi.Led_matrix()
    th_LedMatrix = threading.Thread(target = pi.Led_matrix)
    th_LedMatrix.start()
    time.sleep(0.5)
    

    #蜂鸣器
    th_Buzzer = threading.Thread(target = pi.buzzer)
    th_Buzzer.start()
    time.sleep(0.2)
    
    #震动马达
    th_Vibration = threading.Thread(target = pi.vibration)
    th_Vibration.start()
    time.sleep(0.2)

    #伺服电机
    th_Servo = threading.Thread(target = pi.servo)
    th_Servo.start()
    time.sleep(0.2)

    #步进电机
    th_Step = threading.Thread(target = pi.step)
    th_Step.start()
    time.sleep(0.2)
    
    #继电器
    th_Relay = threading.Thread(target = pi.relay)
    th_Relay.start()
    time.sleep(0.2)

def CloseAll():
    pi.g_exit = True
    print("正在关闭所有模块！")
    os.system("sudo bash kill.sh")


try:
    if __name__ == "__main__":
        outputTest()

except KeyboardInterrupt:
    pi.g_exit = True
