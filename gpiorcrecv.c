/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (C) 2021 Alexander Mishin
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * GPIORCRECV - Remote control receiver module over GPIO.
 *
 * Remote control receiver module can't be discovered automatically, please
 * specify hints s part of loader or kernel configuration:
 *	hint.rcrecv.0.at="gpiobus0"
 *	hint.rcrecv.0.pins=<PIN>
 *
 * Or configure via FDT data.
 */

#define FDT

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/types.h>
#include <sys/module.h>

#include <sys/systm.h>  /* uprintf */
#include <sys/sysctl.h>
#include <sys/conf.h>   /* cdevsw struct */
//#include <sys/param.h>  /* defines used in kernel.h */
#include <sys/kernel.h> /* types used in module initialization */
#include <sys/uio.h>    /* uio struct */
#include <sys/bus.h>

#include <sys/gpio.h>
#include <dev/gpio/gpiobusvar.h>

#ifdef FDT
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
//#include <dev/ofw/ofw_bus_subr.h>

static struct ofw_compat_data compat_data[] = {
    {"rcrecv",   true},
    {NULL,       false}
};

OFWBUS_PNP_INFO(compat_data);
SIMPLEBUS_PNP_INFO(compat_data);
#endif

#define NELEMS(x) (sizeof(x) / sizeof((x)[0]))
/* Use the first/only configured pin. */
#define	PIN_IDX 0

#define RCRECV_CDEV_NAME "rcrecv"

#define SEPARATION_LIMIT 4600
#define RECEIVE_TOLERANCE 60
/* Number of maximum High/Low changes per packet.
 * We can handle up to (unsigned long) => 32 bit * 2 H/L changes per bit + 2 for sync
 */
#define RCSWITCH_MAX_CHANGES 67

struct rcrecv_softc {
    device_t		 dev;
    phandle_t		 node;
    gpio_pin_t		 pin;
    void		*intr_cookie;
    struct resource	*intr_res;
    int			 intr_rid;
    long		 last_evtime;
    size_t		 count_repeat;
    size_t		 count_change;
    unsigned long	 received_value;
    size_t		 received_bit_length;
    size_t		 received_delay;
    int			 received_pulse_length;
    struct cdev		*cdev;
    size_t		 timings[RCSWITCH_MAX_CHANGES];
};

typedef struct {
    uint8_t high;
    uint8_t low;
} levels_ratio;

typedef struct {
    int pulse_length;
    levels_ratio sync_factor;
    levels_ratio zero;
    levels_ratio one;
    bool inverted;
} protocol;

static protocol proto[] = {
    { 350, { 1, 31},  { 1, 3},  { 3, 1},  false },    // protocol 1
    { 650, { 1, 10},  { 1, 2},  { 2, 1},  false },    // protocol 2
    { 100, { 30, 71}, { 4,11},  { 9, 6},  false },    // protocol 3
    { 380, { 1, 6},   { 1, 3},  { 3, 1},  false },    // protocol 4
    { 500, { 6, 14},  { 1, 2},  { 2, 1},  false },    // protocol 5
    { 450, { 23, 1},  { 1, 2},  { 2, 1},  true },     // protocol 6 (HT6P20B)
    { 150, { 2, 62},  { 1, 6},  { 6, 1},  false },    // protocol 7 (HS2303-PT, i. e. used in AUKEY Remote)
    { 200, { 3, 130}, { 7, 16}, { 3, 16}, false },    // protocol 8 Conrad RS-200 RX
    { 200, { 130, 7}, { 16, 7}, { 16, 3}, true },     // protocol 9 Conrad RS-200 TX
    { 365, { 18, 1},  { 3, 1},  { 1, 3},  true },     // protocol 10 (1ByOne Doorbell)
    { 270, { 36, 1},  { 1, 2},  { 2, 1},  true },     // protocol 11 (HT12E)
    { 320, { 36, 1},  { 1, 2},  { 2, 1},  true }      // protocol 12 (SM5212)
};

/* Function prototypes */
static int		rcrecv_probe(device_t);
static int		rcrecv_attach(device_t);
static int		rcrecv_detach(device_t);

static d_open_t		rcrecv_open;
static d_close_t	rcrecv_close;
static d_read_t		rcrecv_read;
static d_write_t	rcrecv_write;
//static d_ioctl_t	rcrecv_ioctl;

/* Character device entry points */
static struct cdevsw rcrecv_cdevsw = {
    .d_version = D_VERSION,
    .d_open = rcrecv_open,
    .d_close = rcrecv_close,
    .d_read = rcrecv_read,
    .d_write = rcrecv_write,
//    .d_ioctl = rcrecv_ioctl,
    .d_name = RCRECV_CDEV_NAME,
};

static inline unsigned int
diff(int A, int B)
{
  return abs(A - B);
}

static bool
rcrecv_receive_protocol(struct rcrecv_softc *sc, const protocol *p)
{
    unsigned long code = 0;
    //Assuming the longer pulse length is the pulse captured in timings[0]
    const size_t sync_length =  ((p->sync_factor.low) > (p->sync_factor.high)) ? (p->sync_factor.low) : (p->sync_factor.high);
    const size_t delay = sc->timings[0] / sync_length;
    const size_t delay_tolerance = delay * RECEIVE_TOLERANCE / 100;

    const size_t first_timing = (p->inverted) ? 2 : 1;

    for (size_t i = first_timing; i < sc->count_change - 1; i += 2)
    {
	code <<= 1;
	if (diff(sc->timings[i], delay * p->zero.high) < delay_tolerance &&
	    diff(sc->timings[i + 1], delay * p->zero.low) < delay_tolerance) {
	    // zero
	}
	else
	if (diff(sc->timings[i], delay * p->one.high) < delay_tolerance &&
	    diff(sc->timings[i + 1], delay * p->one.low) < delay_tolerance) {
	    // one
	    code |= 1;
	}
	else {
	    // Failed
	    return false; 
	}
    } // for (size_t i = 1; i < sc->count_change - 1;

    if (sc->count_change > 7) {    // ignore very short transmissions: no device sends them, so this must be noise
	sc->received_value = code;
	sc->received_bit_length = (sc->count_change - 1) / 2;
	sc->received_delay = delay;
	sc->received_pulse_length = p->pulse_length;
    }

    return true;
}

static void
rcrecv_ihandler(void *arg)
{
    struct rcrecv_softc *sc = arg;

    /* Capture time and pin state first. */
    const long evtime = sbttous(sbinuptime());
    const size_t duration = evtime - sc->last_evtime;

    /* A long stretch without signal level change occurred.
       This could be the gap between two transmission.
     */
    if (duration > SEPARATION_LIMIT)
    {
	/* This long signal is close in length to the long signal which
	   started the previously recorded timings; this suggests that
	   it may indeed by a a gap between two transmissions (we assume
	   here that a sender will send the signal multiple times,
	   with roughly the same gap between them).
	*/
	if ((sc->count_repeat == 0) || (diff(duration, sc->timings[0]) < 200)) {
	    sc->count_repeat++;
	    if (sc->count_repeat == 2) {
		for(size_t i = 0; i < NELEMS(proto); i++) {
		    if (rcrecv_receive_protocol(sc, &(proto[i])))
		    {
#ifdef DEBUG
			device_printf(sc->dev,
			    "proto=>%i, bit=>%i, value=>%lX\n", i, sc->received_bit_length, sc->received_value);
#endif
			break;
		    }
		}
		sc->count_repeat = 0;
	    }
	}

	sc->count_change = 0;
    } //if (duration > SEPARATION_LIMIT)

    /* Detect overflow */
    if (sc->count_change >= RCSWITCH_MAX_CHANGES) {
	sc->count_change = 0;
	sc->count_repeat = 0;
    }

    sc->timings[sc->count_change++] = duration;
    sc->last_evtime = evtime;
}

/* Device _probe() method */
static int
rcrecv_probe(device_t dev)
{
    int rv;

    /*
     * By default we only bid to attach if specifically added by our parent
     * (usually via hint.rcrecv.#.at=busname).  On FDT systems we bid as
     * the default driver based on being configured in the FDT data.
     */
    rv = BUS_PROBE_NOWILDCARD;

#ifdef FDT
    if (ofw_bus_status_okay(dev) &&
	ofw_bus_search_compatible(dev, compat_data)->ocd_data)
	    rv = BUS_PROBE_DEFAULT;
#endif

    device_set_desc(dev, "GPIO RF Receiver module");

    return (rv);
}

static int
rcrecv_detach(device_t dev)
{
    struct rcrecv_softc *sc = device_get_softc(dev);

    if (sc->intr_cookie != NULL)
	bus_teardown_intr(dev, sc->intr_res, sc->intr_cookie);
    if (sc->intr_res != NULL)
	bus_release_resource(dev, SYS_RES_IRQ, sc->intr_rid, sc->intr_res);
    if (sc->pin != NULL)
	gpiobus_release_pin(GPIO_GET_BUS(sc->pin->dev), sc->pin->pin);

    /* Destroy the tm1637 cdev. */
    if (sc->cdev != NULL)
	destroy_dev(sc->cdev);

    return (0);
}

static int
rcrecv_attach(device_t dev)
{
    uint32_t pincaps, edge = GPIO_INTR_EDGE_BOTH;
    int err;

    struct rcrecv_softc *sc = device_get_softc(dev);
    sc->dev = dev;

    struct sysctl_ctx_list	*ctx;
    struct sysctl_oid		*tree_node;
    struct sysctl_oid_list	*tree;

    ctx = device_get_sysctl_ctx(sc->dev);
    tree_node = device_get_sysctl_tree(sc->dev);
    tree = SYSCTL_CHILDREN(tree_node);

    SYSCTL_ADD_ULONG(ctx, tree, OID_AUTO, "value",
	CTLFLAG_RD,
	&sc->received_value, "Value");

    SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "bit_length",
	CTLFLAG_RD,
	&sc->received_bit_length, 0, "Received bit length");

    SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "delay",
	CTLFLAG_RD,
	&sc->received_delay, 0, "Received delay");

    SYSCTL_ADD_INT(ctx, tree, OID_AUTO, "pulse_length",
	CTLFLAG_RD,
	&sc->received_pulse_length, 0, "Received pulse length");

#ifdef FDT
    /* Try to configure our pin from fdt data on fdt-based systems. */
    err = gpio_pin_get_by_ofw_idx(dev, ofw_bus_get_node(dev), PIN_IDX,
	&sc->pin);
#else
    err = ENOENT;
#endif

    /*
     * If we didn't get configured by fdt data and our parent is gpiobus,
     * see if we can be configured by the bus (allows hinted attachment even
     * on fdt-based systems).
     */
    if (err != 0 &&
	strcmp("gpiobus", device_get_name(device_get_parent(dev))) == 0)
	    err = gpio_pin_get_by_child_index(dev, PIN_IDX, &sc->pin);

    /* If we didn't get configured by either method, whine and punt. */
    if (err != 0) {
	device_printf(sc->dev,
	    "cannot acquire gpio pin (config error)\n");
	return (err);
    }

    /* Say what we came up with for pin config. */
    device_printf(dev, "RF input on %s pin %u\n",
	device_get_nameunit(GPIO_GET_BUS(sc->pin->dev)), sc->pin->pin);

    if ((err = gpio_pin_getcaps(sc->pin, &pincaps)) != 0) {
	device_printf(dev, "Cannot query capabilities of gpio pin\n");
	rcrecv_detach(dev);
	return (err);
    }
    if ((pincaps & edge) == 0) {
	device_printf(dev, "Pin cannot be configured for both signal edges\n");
	rcrecv_detach(dev);
	return (ENOTSUP);
    }

    /*
     * Transform our 'gpios' property into an interrupt resource and set up
     * the interrupt.
     */
    if ((sc->intr_res = gpio_alloc_intr_resource(dev, &sc->intr_rid, RF_ACTIVE,
	sc->pin, edge)) == NULL) {
	    device_printf(dev, "Cannot allocate an IRQ for the GPIO\n");
	    rcrecv_detach(dev);
	    return (err);
    }

    err = bus_setup_intr(dev, sc->intr_res, INTR_TYPE_MISC | INTR_MPSAFE,
        NULL, rcrecv_ihandler, sc, &sc->intr_cookie);

    if (err != 0) {
	device_printf(dev, "Unable to setup rf receiver irq handler\n");
	rcrecv_detach(dev);
	return (err);
    }

    /* Create the tm1637 cdev. */
    err = make_dev_p(MAKEDEV_CHECKNAME | MAKEDEV_WAITOK,
	    &sc->cdev,
	    &rcrecv_cdevsw,
	    0,
	    UID_ROOT,
	    GID_WHEEL,
	    0600,
	    RCRECV_CDEV_NAME);

    if (err != 0) {
	device_printf(dev, "Unable to create rcrecv cdev\n");
	rcrecv_detach(dev);
	return (err);
    }

    return (0);
}

static int
rcrecv_open(struct cdev *cdev, int oflags __unused, int devtype __unused,
    struct thread *td __unused)
{

#ifdef DEBUG
	uprintf("Device \"%s\" opened.\n", rcrecv_cdevsw.d_name);
#endif

    return (0);
}

static int
rcrecv_close(struct cdev *cdev __unused, int fflag __unused, int devtype __unused,
    struct thread *td __unused)
{

#ifdef DEBUG
	uprintf("Device \"%s\" closed.\n", rcrecv_cdevsw.d_name);
#endif

    return (0);
}

static int
rcrecv_read(struct cdev *cdev, struct uio *uio, int ioflag __unused)
{
    return (0);
}

static int
rcrecv_write(struct cdev *cdev, struct uio *uio, int ioflag __unused)
{
    return (0);
}

/* Driver bits */
static device_method_t rcrecv_methods[] = {
    /* Device interface */
    DEVMETHOD(device_probe,	rcrecv_probe),
    DEVMETHOD(device_attach,	rcrecv_attach),
    DEVMETHOD(device_detach,	rcrecv_detach),

    DEVMETHOD_END
};

static devclass_t rcrecv_devclass;

DEFINE_CLASS_0(rcrecv, rcrecv_driver, rcrecv_methods, sizeof(struct rcrecv_softc));

#ifdef FDT
DRIVER_MODULE(rcrecv, simplebus, rcrecv_driver, rcrecv_devclass, NULL, NULL);
#endif

DRIVER_MODULE(rcrecv, gpiobus, rcrecv_driver, rcrecv_devclass, 0, 0);
MODULE_VERSION(rcrecv, 1);
