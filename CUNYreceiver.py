#!/usr/bin/env python3

from multiprocessing import Process,Pipe

def async_teensy(child_conn, teensyPort):
    msg = {
        "yes" : 1,
        "no" : 2,
        "maybe" : 3
    }
    
    while True:
        #receivedData, outputArray = receive_from_teensy(teensyPort)
        if receivedData:
            #send_to_teensy(1, 1, teensyPort)
            #child_conn.send(outputArray)
            child_conn.send(msg)
            msg["yes"] += 1
            msg["no"] += 2
            msg["maybe"] += 3
                

    child_conn.close()
