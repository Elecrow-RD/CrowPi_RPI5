#!/bin/sh

pyName=(noPrint manualPrint NFC light ultra DH11 tilt IR)
CRTDIR=$(pwd)
sudo
for i in {0..7};do
	
	pyFile=ageTest_${pyName[i]}.py
	ps -ef|grep $pyFile | grep -v grep|cut -c 9-15|xargs kill -9
done
	sleep 0.2


python3 /home/pi/Desktop/CrowPi/Drivers/aging-test/clear.py
