#!/usr/bin/env python3

from multiprocessing import Process,Pipe

def async_teensy(child_conn, teensyPort):
    msg = 1
    while True:
        receivedData, outputArray = receive_from_teensy(teensyPort)
        if receivedData:
            send_to_teensy(1, 1, teensyPort)
            child_conn.send(outputArray)
                

    child_conn.close()
