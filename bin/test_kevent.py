#!/usr/local/bin/python3.7

import select
from select import kqueue, kevent
import fcntl
import os
import sys
import struct

RCRECV_READ_CODE_INFO = 0x4010520b #1074811403
code_struct = code_buffer = bytearray(4 + 4 * 2 + 1)

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
        """ struct rcrecv_code { unsigned long, size_t, size_t, bool } => 'LII?' """
        code_struct = struct.unpack('LII?', code_buffer)

        code = int(code_struct[0])
        bit_length = int(code_struct[1])
        proto = int(code_struct[2])
        ready = bool(code_struct[3])

        if (bit_length == 24) and code:
            print(hex(code))

kq.close()
os.close(fd)
