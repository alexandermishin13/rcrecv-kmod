# $FreeBSD$

.PATH:	${SRCTOP}/sys/dev/gpio/

KMOD=gpiorcrecv
SRCS=gpiorcrecv.c

SRCS+=	\
	bus_if.h \
	device_if.h \
	gpio_if.h \
	gpiobus_if.h \
	ofw_bus_if.h \
	opt_platform.h \
	fdt_pinctrl_if.h \

CFLAGS+=  -I. -I${SRCTOP}/sys/dev/gpio/
#CFLAGS+=  -DFDT
CFLAGS+=  -DDEBUG

.include <bsd.kmod.mk>
