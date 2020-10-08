def ardno(msg):
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.write(b"{}".format(msg))

    
def send_over_serial(msgArray, serialSend):
    import struct
#IMPORTANT: msgArray NEW FORMAT IN ACCORDANCE WITH ALBORZ COMMUNICATION PROTOCOL
#[ time,
#  LHAX, LHAY, LHAZ, LHGX, LHGY, LHGZ, LHAngle,      RHAX, RHAY, RHAZ, RHGX, RHGY, RHGZ, RHAngle,
#  LSAX, LSAY, LSAZ, LSGX, LSGY, LSGZ, LSAngle,      RSAX, RSAY, RSAZ, RSGX, RSGY, RSGZ, RSAngle,
#  LTAX, LTAY, LTAZ, LTGX, LTGY, LTGZ, LTAngle,      RTAX, RTAY, RTAZ, RTGX, RTGY, RTGZ, RTAngle,
#  LBAX, LBAY, LBAZ, LBGX, LBGY, LBGZ, LBAngle,
#  gaitL, gaitR, slipL, slipR, Torque ]
    sendStr = bytearray(struct.pack("B", 111))
    
    for enum, x in enumerate(msgArray):

        enum == 50 or enum == 51:
            sendStr += bytearray(struct.pack("B", x))
        elif (enum > 0 and enum < 50) or enum > 51:
            sendStr += bytearray(struct.pack("<h", x))
        elif enum == 0:
            sendStr += bytearray(struct.pack("<f", x))
    
    #Encode with UTF-8 and send over serial.
    serialSend.write(sendStr)
    
    
#Cuts number to "digits" number of decimal points.
def truncate(number, digits) -> float:
    import math
    stepper = 10.0 ** digits
    return math.trunc(stepper * number) / stepper
