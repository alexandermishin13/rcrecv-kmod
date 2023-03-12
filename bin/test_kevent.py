#!/usr/local/bin/python3.9

import select
from select import kqueue, kevent
import fcntl
import os
import sys
import struct

""" Build and run ioctl_magic first.
    Copy RCRECV_READ_CODE_INFO magic number and
    recalculate buffer size
"""
RCRECV_READ_CODE_INFO = 0x4020520b #1075859979
code_struct = code_buffer = bytearray(8 + 4 * 4 + 1)

filename = "/dev/rcrecv"
fd = os.open(filename,os.O_RDONLY | os.O_NONBLOCK)
kq = kqueue()

event = [
    kevent(fd,
           filter=select.KQ_FILTER_READ,
           flags=select.KQ_EV_ADD),
]

events = kq.control(event,0,0)

while True:
    r_events = kq.control(None,4)
    for event in r_events:
        fcntl.ioctl(fd, RCRECV_READ_CODE_INFO, code_buffer)
        """ struct rcrecv_code { int64_t, unsigned int, unsigned int, unsigned int, unsigned int, bool } => 'qIIII?' """
        code_struct = struct.unpack('qIIII?', code_buffer)

        last_time = int(code_struct[0])
        bit_length = int(code_struct[1])
        proto = int(code_struct[2])
        code = int(code_struct[3])
        pulse_duration = int(code_struct[4])
        ready = bool(code_struct[5])

        if (bit_length == 24) and code:
            print(hex(code))

kq.close()
os.close(fd)
