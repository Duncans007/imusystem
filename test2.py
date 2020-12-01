#!/usr/bin/env python3

from serialSend import *
import serial
import time

teensyPort = serial.Serial("/dev/ttyACM0", baudrate=256000, timeout=3.0)


while True:
    print("========Start========")
    t1 = time.time()
          
    dataReceived, teensyArray = receive_from_teensy(teensyPort)
    print("From Pi: Teensy data received? ",dataReceived)
    print("From Pi: ",teensyArray)

    if dataReceived:
        torqueL = teensyArray[0]
        torqueR = teensyArray[1]
        send_to_teensy(torqueL, torqueR, teensyPort)
        
    t2 = time.time()-t1
    print("From Pi: period is",t2)
    #line =teensyPort.readline()
    #print(line)
