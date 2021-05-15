#!/usr/local/bin/python3.7

from select import poll, POLLIN

filename = "/dev/rcrecv"

file = open(filename, "r")

p = poll()
p.register(file.fileno(), POLLIN)

while True:
    events = p.poll(2000)
    for e in events:
        print(e)
        # Read data, so that the event goes away?
        print(file.readline(6))
