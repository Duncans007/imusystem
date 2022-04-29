# This file compiles a number of functions for sending data over serial with various packet types.
# Serial packet structure is defined and commented within each function.



def receive_from_teensy(serialPort):
    import struct
#[ 165, 90, LENGTH(101),
# Actual Torque L, Actual Torque R, Knee Angle L, Knee Angle R
# LT angX, LT angY, LT angZ, LT gyX, LT gyY, LT gyZ, LT acX, LT acY, LT acZ
# RT angX, RT angY, RT angZ, RT gyX, RT gyY, RT gyZ, RT acX, RT acY, RT acZ
# LS angX, LS angY, LS angZ, LS gyX, LS gyY, LS gyZ, LS acX, LS acY, LS acZ
# RS angX, RS angY, RS angZ, RS gyX, RS gyY, RS gyZ, RS acX, RS acY, RS acZ
# BB angX, BB angY, BB angZ, BB gyX, BB gyY, BB gyZ, BB acX, BB acY, BB acZ]

    receivedData = False
    outputArray = [None] * 49
    
    firstChar = serialPort.read() #Byte 1
    try:
        firstCharInt = struct.unpack('B', firstChar)
    except:
        firstCharInt = (0,0)
        
    if (firstCharInt[0] == 165):
        secondChar = serialPort.read() #Byte 2
        secondCharInt = struct.unpack('B', secondChar)
        if (secondCharInt[0] == 90):
            
            dataSize = serialPort.read() #Byte 3
            dataSizeInt = struct.unpack('B',dataSize)
            
            recArray = [None] * 49
            
            
            for x in range(49):
                loByte = serialPort.read()
                hiByte = serialPort.read()
                bytesTemp = struct.unpack('<h', hiByte + loByte)
                recArray[x] = bytesTemp[0]
                outputArray[x] = recArray[x]/100.0#-32768
            
            receivedData = True
    
    return receivedData, outputArray





#---------------------------------------------------------------------------------------






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
    
    teest1 = int(torqueLeft * 100)
    #teest2 = int((torqueLeft * 100)%256)
    teest3 = int(torqueRight * 100)
    #teest4 = int((torqueRight * 100)%256)
    sendStr = bytearray(struct.pack("B", 165))
    sendStr += bytearray(struct.pack("B", 90))
    sendStr += bytearray(struct.pack("B", 52))
    sendStr += bytearray(struct.pack("<h", int(torqueLeft * 100)))
    #sendStr += bytearray(struct.pack("c", teest2)
    sendStr += bytearray(struct.pack("<h", int(torqueRight * 100)))
    #sendStr += bytearray(struct.pack("c", teest4)
    #teest1 = int((torqueLeft * 100)//256)
    #print(sendStr)
    #print(int(torqueLeft * 100))
    #print(int(torqueRight * 100))
    serialPort.write(sendStr)





#---------------------------------------------------------------------------------------


def send_to_brace(gait, serialPort):
    import struct
    import serial
    import time
    
    sendStr = bytearray(struct.pack("B", int(gait)))
    serialPort.write(sendStr)





#---------------------------------------------------------------------------------------




# Send to NUC    
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

    if len(msgArray) < 60:
        sendStr = bytearray(struct.pack("B", 113))
    else:
        sendStr = bytearray(struct.pack("B", 113 + 98))
    
    for enum, x in enumerate(msgArray):

        if enum == 50 or enum == 51:
            sendStr += bytearray(struct.pack("B", x))
        elif (enum > 0 and enum < 50) or enum > 51:
            sendStr += bytearray(struct.pack("<h", int(x)))
        elif enum == 0:
            sendStr += bytearray(struct.pack("<f", int(x)))
    
    #Encode with UTF-8 and send over serial.
    serialSend.write(sendStr)
    




#---------------------------------------------------------------------------------------





    
def receive_from_nuc(serialPort):
    import struct
#[LENGTH(5),
# Knee Angle L, Knee Angle R]

    receivedData = False
    outputArray = []
    
    firstChar = serialPort.read() #Byte 1
    try:
        firstCharInt = struct.unpack('B', firstChar)
    except:
        firstCharInt = (0,0)
    

    if (firstCharInt[0] == 165):
        recArray = []
            
        loByte = serialPort.read()
        hiByte = serialPort.read()
        recArray += struct.unpack('<h', loByte + hiByte)
        
        loByte = serialPort.read()
        hiByte = serialPort.read()
        recArray += struct.unpack('<h', loByte + hiByte)
        
        loByte = serialPort.read()
        hiByte = serialPort.read()
        recArray += struct.unpack('<h', loByte + hiByte)
            
        receivedData = True
        outputArray = recArray
    
    return receivedData, outputArray





#---------------------------------------------------------------------------------------






def receive_from_arduino(serialPort):
    import struct
#[165, val loByte, val hiByte]

    receivedData = False
    outputVal = 0
    
    firstChar = serialPort.read() #Byte 1
    try:
        firstCharInt = struct.unpack('B', firstChar)
    except:
        firstCharInt = (0,0)
    

    if (firstCharInt[0] == 165):
        loByte = serialPort.read()
        hiByte = serialPort.read()
        outputVal = struct.unpack('<h', loByte + hiByte)

        receivedData = True
    
    return receivedData, outputVal
