import serial
import re
import time 

def jacksonPingIR():
    ser = serial.Serial('/dev/ttyUSB0',115200)
    flag = 1
    while flag:
        if(ser.in_waiting>0):
            line = ser.readline()
            try:
                distance = int(line)
                distance = distance/100
                print(distance)
                flag = 0
                return distance
            except:
                print("BAD VALUE")
                pass
            	
#firstNum = jacksonPingIR()			#I can update how fast this process will take by changing the arduino code where it has the delay function
#time.sleep(1)
#secondNum = jacksonPingIR()
#jacksonPingIR()
#jacksonPingIR()
#jacksonPingIR()
#total = firstNum+secondNum
#print(total)
