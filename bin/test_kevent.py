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
        """ struct rcrecv_code { int64_t, unsigned long, size_t, size_t, bool } => 'qLNN?' """
        code_struct = struct.unpack('qLNN?', code_buffer)

        last_time = int(code_struct[0])
        code = int(code_struct[1])
        bit_length = int(code_struct[2])
        proto = int(code_struct[3])
        ready = bool(code_struct[4])

        if (bit_length == 24) and code:
            print(hex(code))

kq.close()
os.close(fd)
