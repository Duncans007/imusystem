#!/usr/bin/env python3

from multiprocessing import Process,Queue,Pipe
from testChild import f

if __name__ == '__main__':
    parent_conn,child_conn = Pipe()
    p = Process(target=f, args=(child_conn,))
    p.start()
    print(parent_conn.recv())   # prints "Hello"
