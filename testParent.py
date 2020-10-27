#!/usr/bin/env python3

from multiprocessing import Process,Queue,Pipe
import time
from CUNYreceiver import async_teensy

if __name__ == '__main__':
    parent_conn,child_conn = Pipe()
    p = Process(target=async_teensy, args=(child_conn,))
    p.start()
    while True:
        print(parent_conn.recv())   # prints "Hello"
        time.sleep(1/50)
