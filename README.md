# gpiorcrecv-kmod

FreeBSD kernel module for GPIO remote control receiver

## About



## Installation

You need the FreeBSD source codes for build the driver. You can copy it,
mount usb-flash with it or mount it as NFS share from another PC to
`/usr/src` as I did - It doesn't matter - all these methods will work fine.
Run this commands from driver directory:
```
make
sudo make install
```
Now You installed the driver You also need to define the receiver as a device.
Go to `./fdt-overlay` folder and choose an example of `.dtso` overlay that
suits Your system best. Copy it to a name without `.sample` tail and edit it.
Lines You should pay attention to are:
```
compatible =
pins =
gpios =
```
Obviously You should to define the pin You connect Your receiver to. Also You
need to define a right compatible string (Simplest way to do it is take the
same one from Your other overlays).
To build and install Your new overlay run:
```
make
sudo make install
```
All You need now is to add a name of new fdt-blob to `/boot/loader.conf` for
autoload it when system is rebooted (An extension can be omitted):
```
fdt_overlays="your,other,overlays,sun8i-h3-rcrecv-gpio"
```

## Status

Finshed but still under development
