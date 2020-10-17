def ardno(msg):
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.write(b"{}".format(msg))

    
def receive_from_teensy(serialPort):
    receivedData = False
    outputArray = []
    
    firstChar = serialPort.read() #Byte 1
    
    if (firstChar == 165):
        secondChar = serialPort.read() #Byte 2
        if (secondChar == 90):
            dataSize = serialPort.read() #Byte 3
            dataSizeInt = struct.unpack('B',dataSize)
            
            LTorqueLO = serialPort.read() #Byte 4
            LTorqueHI = serialPort.read() #Byte 5
            LTorque = struct.unpack('<h', LTorqueLO + LTorqueHI)
            
            RTorqueLO = serialPort.read() #Byte 6
            RTorqueHI = serialPort.read() #Byte 7
            RTorque = struct.unpack('<h', RTorqueLO + RTorqueHI)
            
            LKneeAngleLO = serialPort.read() #Byte 8
            LKneeAngleHI = serialPort.read() #Byte 9
            LKneeAngle = struct.unpack('<h', LKneeAngleLO + LKneeAngleHI)
            
            RKneeAngleLO = serialPort.read() #Byte 10
            RKneeAngleHI = serialPort.read() #Byte 11
            RKneeAngle = struct.unpack('<h', RKneeAngleLO + RKneeAngleHI)
            
            LTAngleLO = serialPort.read() #Byte 12
            LTAngleHI = serialPort.read() #Byte 13
            LTAngle = struct.unpack('<h', LTAngleLO + LTAngleHI)
            
            RTAngleLO = serialPort.read() #Byte 14
            RTAngleHI = serialPort.read() #Byte 15
            RTAngle = struct.unpack('<h', RTAngleLO + RTAngleHI)
            
            LSAngleLO = serialPort.read() #Byte 16
            LSAngleHI = serialPort.read() #Byte 17
            LSAngle = struct.unpack('<h', LSAngleLO + LSAngleHI)
            
            RSAngleLO = serialPort.read() #Byte 18
            RSAngleHI = serialPort.read() #Byte 19
            RSAngle = struct.unpack('<h', RSAngleLO + RSAngleHI)
            
            LTangVLO = serialPort.read() #Byte 20
            LTangVHI = serialPort.read() #Byte 21
            LTangV = struct.unpack('<h', LTangVLO + LTangVHI)
            
            RTangVLO = serialPort.read() #Byte 22
            RTangVHI = serialPort.read() #Byte 23
            RTangV = struct.unpack('<h', RTangVLO + RTangVHI)
            
            LSangVLO = serialPort.read() #Byte 24
            LSangVHI = serialPort.read() #Byte 25
            LSangV = struct.unpack('<h', LSangVLO + LSangVHI)
            
            RSangVLO = serialPort.read() #Byte 26
            RSangVHI = serialPort.read() #Byte 27
            RSangV = struct.unpack('<h', RSangVLO + RSangVHI)
            
            outputArray = [LTorque, RTorque, LKneeAngle, RKneeAngle, LTAngle, RTAngle, LSAngle, RSAngle, LTangV, RTangV, LSangV, RSangV]
            receivedData = True
            print(outputArray)
    
    return receivedData, outputArray
    
def send_to_teensy(torqueLeft, torqueRight, serialPort):
    import struct
    import serial
    import time
    
    #Bytes (7 total):
    #0: 165
    #1: 90
    #2: data length (4 bytes)
    #3: left torque high byte
    #4: left torque low byte
    #5: right torque high byte
    #6: right torque low byte
    
    sendStr = bytearray(struct.pack("B", 165))
    sendStr += bytearray(struct.pack("B", 90))
    sendStr += bytearray(struct.pack("B", 52))
    sendStr += bytearray(struct.pack("<H", int(torqueLeft * 1000)))
    sendStr += bytearray(struct.pack("<H", int(torqueRight * 1000)))
    serialPort.write(sendStr)

    
    
def send_over_serial(msgArray, serialSend):
    import struct
#IMPORTANT: msgArray NEW FORMAT IN ACCORDANCE WITH ALBORZ COMMUNICATION PROTOCOL
#[ time,
#  LHAX, LHAY, LHAZ, LHGX, LHGY, LHGZ, LHAngle,      RHAX, RHAY, RHAZ, RHGX, RHGY, RHGZ, RHAngle,
#  LSAX, LSAY, LSAZ, LSGX, LSGY, LSGZ, LSAngle,      RSAX, RSAY, RSAZ, RSGX, RSGY, RSGZ, RSAngle,
#  LTAX, LTAY, LTAZ, LTGX, LTGY, LTGZ, LTAngle,      RTAX, RTAY, RTAZ, RTGX, RTGY, RTGZ, RTAngle,
#  LBAX, LBAY, LBAZ, LBGX, LBGY, LBGZ, LBAngle,
#  gaitL, gaitR, slipL, slipR, TorqueL, TorqueR ]

#To extract values:
#Torque = x * 0.002
#Angle = x * .0125
#Accelerometer = x * 0.002394
#Gyroscope = x * 0.07


    sendStr = bytearray(struct.pack("B", 113))
    
    for enum, x in enumerate(msgArray):

        if enum == 50 or enum == 51:
            sendStr += bytearray(struct.pack("B", x))
        elif (enum > 0 and enum < 50) or enum > 51:
            sendStr += bytearray(struct.pack("<h", int(x)))
        elif enum == 0:
            sendStr += bytearray(struct.pack("<f", x))
    
    #Encode with UTF-8 and send over serial.
    serialSend.write(sendStr)
    
    
#Cuts number to "digits" number of decimal points.
def truncate(number, digits) -> float:
    import math
    stepper = 10.0 ** digits
    return math.trunc(stepper * number) / stepper
