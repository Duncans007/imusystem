#!/usr/bin/env python3

# This function is used to create an asynchronous listener that can be called by the main program. It constantly listens to data input from the Serial input configured for arduino. Typically used for Load Cell data.

from multiprocessing import Process,Pipe
from serialSend import *

def async_arduino(child_conn, arduinoPort):
    output_value = 0
    
    while True:
        receivedData, output_value = receive_from_arduino(arduinoPort)
        if receivedData:
            child_conn.send(output_value)
                
    child_conn.close()
