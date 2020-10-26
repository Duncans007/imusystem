#!/usr/bin/env python3

from multiprocessing import Process,Pipe

def f(child_conn):
    msg = 1
    while True:
        msg = msg + 1
        child_conn.send(msg)
    child_conn.close()
