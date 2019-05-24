# -*-coding:Latin-1 -*
import os
import csv

import serial # you need to install the pySerial :pyserial.sourceforge.net
import time
# your Serial port should be different!
arduino = serial.Serial('COM45', 115200)


print("Demarrage")
f = open('donnees_TP.csv','w')

try:
    #writer = csv.writer(f)
    arduino.write(str.encode('p'))
    lineread = arduino.readline().decode("utf-8")
    line = arduino.readline().decode("utf-8")
    f.write("temp" + "\t" + "press" + "\t" + "n" + '\n')
    x = 1
    while True :
        print(line)
        f.write(line)
        if "End of Read" in line:
            break
        line = arduino.readline().decode("utf-8") 
        
finally:
    f.close()

