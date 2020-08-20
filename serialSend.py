def ardno(msg):
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.write(b"{}".format(msg))

    
def send_over_serial(msgArray, serialSend):
    sendStr = ""
    
    #Pools sensor values into string for conversion to bytes
    for n in msgArray:    
        try:
            x = truncate(n, 2.0)
        except TypeError:
            x = n
        sendStr += f"{x},"
    
    #Cut last comma, add terminating character instead
    sendStr = sendStr[:-1]
    sendStr += f"\n"
    
    #Encode with UTF-8 and send over serial.
    serialSend.write(sendStr.encode('utf-8'))
    
    
#Cuts number to "digits" number of decimal points.
def truncate(number, digits) -> float:
    import math
    stepper = 10.0 ** digits
    return math.trunc(stepper * number) / stepper
