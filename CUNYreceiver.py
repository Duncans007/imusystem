#!/usr/bin/env python3

# This function is used to receive and send data to the device provided for kneeling research by CUNY.

from multiprocessing import Process,Pipe
from serialSend import *

def async_teensy(child_conn, teensyPort):
    cuny_data = {
        "ActTqL" : 0,
        "ActTqR" : 0,
        "KneeL"  : 0,
        "KneeR"  : 0,
        "LTangX" : 0,
        "LTangY" : 0,
        "LTangZ" : 0,
        "LTgyX"  : 0,
        "LTgyY"  : 0,
        "LTgyZ"  : 0,
        "LTacX"  : 0,
        "LTacY"  : 0,
        "LTacZ"  : 0,
        "RTangX" : 0,
        "RTangY" : 0,
        "RTangZ" : 0,
        "RTgyX"  : 0,
        "RTgyY"  : 0,
        "RTgyZ"  : 0,
        "RTacX"  : 0,
        "RTacY"  : 0,
        "RTacZ"  : 0,
        "LSangX" : 0,
        "LSangY" : 0,
        "LSangZ" : 0,
        "LSgyX"  : 0,
        "LSgyY"  : 0,
        "LSgyZ"  : 0,
        "LSacX"  : 0,
        "LSacY"  : 0,
        "LSacZ"  : 0,
        "RSangX" : 0,
        "RSangY" : 0,
        "RSangZ" : 0,
        "RSgyX"  : 0,
        "RSgyY"  : 0,
        "RSgyZ"  : 0,
        "RSacX"  : 0,
        "RSacY"  : 0,
        "RSacZ"  : 0,
        "BangX"  : 0,
        "BangY"  : 0,
        "BangZ"  : 0,
        "BgyX"   : 0,
        "BgyY"   : 0,
        "BgyZ"   : 0,
        "BacX"   : 0,
        "BacY"   : 0,
        "BacZ"   : 0
    }
    
    while True:
        receivedData, outputArray = receive_from_teensy(teensyPort)
        if receivedData:
            
            try:
                cuny_data["ActTqL"] = outputArray[0]
                cuny_data["ActTqR"] = outputArray[1]
                cuny_data["KneeL"]  = outputArray[2]
                cuny_data["KneeR"]  = outputArray[3]
                cuny_data["LTangX"] = outputArray[4]
                cuny_data["LTangY"] = outputArray[5]
                cuny_data["LTangZ"] = outputArray[6]
                cuny_data["LTgyX"]  = outputArray[7]
                cuny_data["LTgyY"]  = outputArray[8]
                cuny_data["LTgyZ"]  = outputArray[9]
                cuny_data["LTacX"]  = outputArray[10]
                cuny_data["LTacY"]  = outputArray[11]
                cuny_data["LTacZ"]  = outputArray[12]
                cuny_data["RTangX"] = outputArray[13]
                cuny_data["RTangY"] = outputArray[14]
                cuny_data["RTangZ"] = outputArray[15]
                cuny_data["RTgyX"]  = outputArray[16]
                cuny_data["RTgyY"]  = outputArray[17]
                cuny_data["RTgyZ"]  = outputArray[18]
                cuny_data["RTacX"]  = outputArray[19]
                cuny_data["RTacY"]  = outputArray[20]
                cuny_data["RTacZ"]  = outputArray[21]
                cuny_data["LSangX"] = outputArray[22]
                cuny_data["LSangY"] = outputArray[23]
                cuny_data["LSangZ"] = outputArray[24]
                cuny_data["LSgyX"]  = outputArray[25]
                cuny_data["LSgyY"]  = outputArray[26]
                cuny_data["LSgyZ"]  = outputArray[27]
                cuny_data["LSacX"]  = outputArray[28]
                cuny_data["LSacY"]  = outputArray[29]
                cuny_data["LSacZ"]  = outputArray[30]
                cuny_data["RSangX"] = outputArray[31]
                cuny_data["RSangY"] = outputArray[32]
                cuny_data["RSangZ"] = outputArray[33]
                cuny_data["RSgyX"]  = outputArray[34]
                cuny_data["RSgyY"]  = outputArray[35]
                cuny_data["RSgyZ"]  = outputArray[36]
                cuny_data["RSacX"]  = outputArray[37]
                cuny_data["RSacY"]  = outputArray[38]
                cuny_data["RSacZ"]  = outputArray[39]
                cuny_data["BangX"]  = outputArray[40]
                cuny_data["BangY"]  = outputArray[41]
                cuny_data["BangZ"]  = outputArray[42]
                cuny_data["BgyX" ]  = outputArray[43]
                cuny_data["BgyY" ]  = outputArray[44]
                cuny_data["BgyZ" ]  = outputArray[45]
                cuny_data["BacX" ]  = outputArray[46]
                cuny_data["BacY" ]  = outputArray[47]
                cuny_data["BacZ" ]  = outputArray[48]
            except IndexError:
                continue
            
            child_conn.send(cuny_data)
        else:
            child_conn.send(cuny_data)
                

    child_conn.close()
