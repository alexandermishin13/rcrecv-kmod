#!/usr/local/bin/python3.7

import time

if __name__ == "__main__":
    f = open("/dev/rcrecv", "r")
    while True:
        code = f.readline()
        if code:
            print(code)
        else:
            time.sleep(1)
    f.close()
