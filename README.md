# rcrecv-kmod

FreeBSD kernel module for GPIO remote control receiver

<img width=240 height=180 src="img/RC_Receiver_433MHz.jpeg" title="RC receiver 433MHz" />

## About

The kernel driver reads a sequence of pulses from a remote control receiver
(e.g., mx-rm-5v) and tries to decode it into a number. This number can be read
from a character device or received by ioctl() as a remote control code. The
driver also generates `poll`(2) and `kqueue`(2) events informing the user
process about the presence of new code for it.
It is possible to read additional information about the code using a ioctl()
call (such as a timestamp. See `rcrecv.h` and `./bin/` folder for examples).

## Installation

You need the FreeBSD source codes for build the driver. You can copy it or
mount usb-flash with it or mount it as NFS share from another PC to
`/usr/src` as I did - It doesn't matter - all these methods will work fine.
Run this commands from driver directory (If Your platform haven't DTS enabled
You can skip the first line to keep `opt_platform.h` empty):
```shell
% echo "#define FDT 1" > opt_platform.h
% make
% sudo make install
```
Now You installed the driver You also need to define the receiver as a device.
You can do it either by FDT-overlay or by device.hints.

### FDT-overlay based setup

Go to `./fdt-overlay` folder and choose an example of `.dtso` overlay that
suits Your system best. Copy it to a name without `.sample` tail and edit it.
Lines to pay attention to:
```ini
compatible =
pins =
gpios =
```
Obviously You should define the pin You connect Your receiver to. You also
have to set correctly the compatibility string (The implest way to do this is
to take the one from Your other overlays).

To build and install Your new overlay run:
```shell
% make
% sudo make install
```
All You have to do now is add a name of new fdt-blob to `/boot/loader.conf` to
have it automatically loaded on system reboot (An extension can be omitted):
```ini
fdt_overlays="your,other,overlays,sun8i-h3-rcrecv-gpio"
```

### device.hints based pin setup

For not DTS-compatible system You can setup the device by editing a file
`/boot/device.hints`. By example, for a device on pin 13:

```ini
hint.rcrecv.0.at="gpiobus0"
hint.rcrecv.0.compatible="rcrecv"
hint.rcrecv.0.pin_list="13"
### Optional. 60% is default tolerance value
hint.rcrecv.0.default-tolerance="60"
```

After changes are made You need to reboot Your system - `kenv` command
changes the variables ok but it seems they does not be honored on already
running system anyway.

## Status

Tested on `Orange PI PC`, `Orange PI Zero`, `Raspberry Pi 2b`, `Raspberry Pi 4b`.
Works like a charm for me.

## Thanks

Suat Özgür (aka sui77) for [rc-switch](https://github.com/sui77/rc-switch)
