def ardno(msg):
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()
    ser.write(b"{}".format(msg))

    
def send_over_serial(msgArray, serialSend):
    sendStr = ""
    
    #Pools sensor values into string for conversion to bytes
    for n in msgArray:
        sendStr += f"{truncate(n,2)},"
    
    #Cut last comma, add terminating character instead
    sendStr = sendStr[:-1]
    sendStr += f"\n"
    
    #Encode with UTF-8 and send over serial.
    serialSend.flush()
    serialSend.write(sendStr.encode('utf-8'))
    
    
#Cuts number to "digits" number of decimal points.
def truncate(number, digits) -> float:
    import math
    stepper = 10.0 ** digits
    return math.trunc(stepper * number) / stepper
