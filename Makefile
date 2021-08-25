# $FreeBSD$

KMOD=rcrecv
SRCS=rcrecv.c
SUBDIR=include man

SRCS+=	\
	bus_if.h \
	device_if.h \
	gpio_if.h \
	gpiobus_if.h \
	ofw_bus_if.h \
	opt_platform.h \
	fdt_pinctrl_if.h \

CFLAGS+=  -I.

.include <bsd.kmod.mk>
