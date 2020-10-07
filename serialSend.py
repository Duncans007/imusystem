def ardno(msg):
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.write(b"{}".format(msg))

    
    
def send_to_teensy(serialPort, torqueLeft, torqueRight):
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
    sendStr += bytearray(struct.pack("<h", torqueLeftCorrected))
    sendStr += bytearray(struct.pack("<h", torqueRightCorrected))
    serialPort.write(sendStr)

    
    
def send_over_serial(msgArray, serialSend):
    sendStr = ""
    
    #Pools sensor values into string for conversion to bytes
    for n in msgArray:    
        try:
            x = truncate(n, 5.0)
        except TypeError:
            x = n
        sendStr += f"{x},"
    
    #Cut last comma, add terminating character instead
    sendStr = sendStr[:-1]
    sendStr += f"\n"
    
    if msgArray[0] == "PR":
        sendStr += f"\r"
    
    #Encode with UTF-8 and send over serial.
    serialSend.write(sendStr.encode('utf-8'))
    
    
#Cuts number to "digits" number of decimal points.
def truncate(number, digits) -> float:
    import math
    stepper = 10.0 ** digits
    return math.trunc(stepper * number) / stepper
