# gpiorcrecv-kmod

FreeBSD kernel module for GPIO remote control receiver

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
Go to `./fdt-overlay` folder and choose an `.dtso` overlay that suits Your
system best. Surely, You need to edit it. Lines You should pay attention to
are:
```
compatible =
pins =
gpios =
```
Obviously You should to set a pin You will use for the reciever and set a right
compatible string (take one from Your other overlays). I recommend You to copy 
this selected overlay to another file with a new name and edit that new one.
To build and install Your new overlay run:
```
make
sudo make install
```
All You need now is to add new fdt-blob to `/boot/loader.conf` for autoload it
when system is rebooted:
```
fdt_overlays="your,other,overlays,sun8i-h3-rcrecv-gpio"
```

## Status

* Gets the code from a remote control;
* Sets kernel variables;
* Creates a device file and read a last code from it;

Under early development
