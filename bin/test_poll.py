#!/usr/local/bin/python3.8

from select import poll, POLLIN

filename = "/dev/rcrecv"

file = open(filename, "r")

p = poll()
p.register(file.fileno(), POLLIN)

while True:
    events = p.poll(10000)
    for e in events:
        print(e)
        # Read data, so that the event goes away?
        print(file.readline())
