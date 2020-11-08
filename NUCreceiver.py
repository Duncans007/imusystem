#!/usr/bin/env python3

from multiprocessing import Process,Pipe
from serialSend import *

def async_nuc(child_conn, nucPort):
    nuc_data = {
        "KneeL"  : 0,
        "KneeR"  : 0,
        "Back"   : 0

    }
    
    while True:
        receivedData, outputArray = receive_from_nuc(nucPort)
        if receivedData:
            
            try:
                nuc_data["KneeL"]  = outputArray[0]
                nuc_data["KneeR"]  = outputArray[1]
                nuc_data["Back"] = outputArray[2]
            except IndexError:
                continue
            
            

        child_conn.send(nuc_data)
                

    child_conn.close()
