def ardno(msg):
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()
    ser.write(b"{}".format(msg))

    
def send_over_serial(msgArray, serialSend):
    sendStr = ""
    for n in msgArray:
        sendStr += f"{truncate(n,2)},"
    
    sendStr = sendStr[:-1]
    sendStr += f"\n"
    
    serialSend.flush()
    serialSend.write(sendStr.encode('utf-8'))
    
    

def truncate(number, digits) -> float:
    import math
    stepper = 10.0 ** digits
    return math.trunc(stepper * number) / stepper
