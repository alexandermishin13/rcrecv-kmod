#!/usr/local/bin/python3.7

import time
import fcntl
import os
import struct

GPIOC_DEV = '/dev/rcrecv'
RCRECV_READ_CODE = 0x4004520a #1074024970

""" For 32 bit OS
    sizeof(unsigned long) == 4
"""
code_struct = code_buffer = bytearray(4)

if __name__ == "__main__":
    # You need to open the device, not the link
    if os.path.islink(GPIOC_DEV):
        device_path = os.readlink(GPIOC_DEV)
        if not device_path.startswith('/'):
            device_path = os.path.dirname(GPIOC_DEV) + '/' + device_path
    else:
        device_path = GPIOC_DEV

    device_fd = os.open(device_path, os.O_RDONLY | os.O_NONBLOCK )

    while True:
        fcntl.ioctl(device_fd, RCRECV_READ_CODE, code_buffer)
        code_struct = struct.unpack('l', code_buffer)
        code = int(code_struct[0])

        if code:
            print(hex(code))

        time.sleep(1)
    os.close(device_fd)
