#!/usr/local/bin/python3.7

import select
from select import kqueue, kevent
import os
import sys

filename = "/dev/rcrecv"
fd = os.open(filename,os.O_RDONLY)
kq = kqueue()

event = [
    kevent(fd,
           filter=select.KQ_FILTER_READ,
           flags=select.KQ_EV_ADD),
]

events = kq.control(event,0,0)

while True:
    print("loop")
    r_events = kq.control(None,4)
    for event in r_events:
        print(event)
        print(os.read(fd, event.data))

kq.close()
os.close(fd)