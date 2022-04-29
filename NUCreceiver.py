#!/usr/bin/env python3

from multiprocessing import Process,Pipe
from serialSend import *

def async_nuc(child_conn, nucPort):
    nuc_data = {
        "L"  : 0,
        "R"  : 0,
        "B"   : 0

    }
    
    while True:
        receivedData, outputArray = receive_from_nuc(nucPort)
        if receivedData:
            
            try:
                nuc_data["L"]  = outputArray[0]
                nuc_data["R"]  = outputArray[1]
                nuc_data["B"] = outputArray[2]
            except IndexError:
                continue
            
            child_conn.send(nuc_data)
        else:
            child_conn.send(nuc_data)
                

    child_conn.close()
