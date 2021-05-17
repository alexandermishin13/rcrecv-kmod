#!/usr/local/bin/python3.7

import time
import fcntl
import os
import struct

GPIOC_DEV = '/dev/rcrecv'
RCRECV_READ_CODE_INFO = 0x400c520b #1074549259

""" For 32 bit OS
    struct rcrecv_code { unsigned long, size_t, size_t }
    sizeof(struct rcrecv_code) = 4 + 4 * 2
"""
code_struct = code_buffer = bytearray(4 + 4 * 2 + 1)

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
        fcntl.ioctl(device_fd, RCRECV_READ_CODE_INFO, code_buffer)
        """ struct rcrecv_code { unsigned long, size_t, size_t, bool } => 'LII?' """
        code_struct = struct.unpack('LII?', code_buffer)

        code = int(code_struct[0])
        bit_length = int(code_struct[1])
        proto = int(code_struct[2])
        ready = bool(code_struct[3])

        if (bit_length == 24) and code:
            print(hex(code))

        time.sleep(1)
    os.close(device_fd)
