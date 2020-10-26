#!/usr/bin/env python3

from multiprocessing import Process,Pipe

def async_teensy(child_conn):
    msg = {
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
        #receivedData, outputArray = receive_from_teensy(teensyPort)
        #if receivedData:
            #send_to_teensy(1, 1, teensyPort)
            #child_conn.send(outputArray)
        child_conn.send(msg)
                

    child_conn.close()
