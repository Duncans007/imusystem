#!/usr/bin/env python3

from multiprocessing import Process,Pipe
from serialSend import *

def async_arduino(child_conn, arduinoPort):
    output_value = 0
    
    while True:
        receivedData, output_value = receive_from_arduino(teensyPort)
        if receivedData:
            child_conn.send(output_value)
        else:
            child_conn.send(-1)
                
    child_conn.close()
