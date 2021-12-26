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
 * RCRECV - Remote control receiver module over GPIO.
 *
 * Remote control receiver module can't be discovered automatically, please
 * specify hints s part of loader or kernel configuration:
 *	hint.rcrecv.0.at="gpiobus0"
 *	hint.rcrecv.0.pins=<PIN>
 *
 * Or configure via FDT data.
 */

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

#include "include/dev/rcrecv/rcrecv.h"

#ifdef FDT
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#endif

#include <sys/mutex.h>
#include <sys/selinfo.h>
#include <sys/poll.h>

#ifdef FDT
static struct ofw_compat_data compat_data[] = {
    {"rcrecv",   true},
    {NULL,       false}
};

OFWBUS_PNP_INFO(compat_data);
SIMPLEBUS_PNP_INFO(compat_data);
#endif

#define NELEMS(x) \
    (sizeof(x) / sizeof((x)[0]))

/* Use the first/only configured pin. */
#define	PIN_IDX 0

#define RCRECV_CDEV_NAME "rcrecv"

#define SEPARATION_GAP_LIMIT 4600
#define SEPARATION_GAP_DELTA 200
#define RECEIVE_TOLERANCE 60
/* Number of maximum rising/falling edges per packet.
 * We can handle up to (uint32_t) => 32 bit * 2 edges per bit + 2 edges for sync
 */
#define RCSWITCH_MAX_CHANGES 67

static const size_t OFFSET_A = 'a' - '9' - 1;

MALLOC_DEFINE(M_RCRECVCODE, "rcrecvcode", "Struct for received code info");
MALLOC_DEFINE(M_RCRECVSEQ, "rcrecvseq", "Struct for received edges sequence");

struct rcrecv_seq {
    size_t		 edges_count;
    size_t		 timings[RCSWITCH_MAX_CHANGES];
    bool		 completed;
};

struct rcrecv_softc {
    device_t		 dev;
    struct cdev		*cdev;
    void		*intr_cookie;
    struct resource	*intr_res;
    struct rcrecv_code	*rc_code;
    struct rcrecv_seq	*rc_seq;
    int64_t		 last_evtime;
    gpio_pin_t		 pin;
    size_t		 edges_count_min;
    struct selinfo	 rsel;
    struct mtx		 mtx;
    int			 intr_rid;
    uint8_t		 receive_tolerance;
    bool		 poll_sel;
    char		 received_code[sizeof(((struct rcrecv_code *)0)->value) * 2 + 1]; // two chars per byte + '\0'
};

static d_open_t		rcrecv_open;
static d_close_t	rcrecv_close;
static d_read_t		rcrecv_read;
static d_ioctl_t	rcrecv_ioctl;
static d_poll_t		rcrecv_poll;
static d_kqfilter_t	rcrecv_kqfilter;
static int		rcrecv_kqevent(struct knote *, long);
static void		rcrecv_kqdetach(struct knote *);

static struct filterops rcrecv_filterops = {
    .f_isfd =		1,
    .f_attach =		NULL,
    .f_detach =		rcrecv_kqdetach,
    .f_event =		rcrecv_kqevent,
};

/* Function prototypes */
static int		rcrecv_probe(device_t);
static int		rcrecv_attach(device_t);
static int		rcrecv_detach(device_t);
static void		rcrecv_notify(struct rcrecv_softc *);

static int		rcrecv_tolerance_sysctl(SYSCTL_HANDLER_ARGS);

/* Character device entry points */
static struct cdevsw rcrecv_cdevsw = {
    .d_version =	D_VERSION,
    .d_open =		rcrecv_open,
    .d_close =		rcrecv_close,
    .d_read =		rcrecv_read,
    .d_ioctl =		rcrecv_ioctl,
    .d_poll =		rcrecv_poll,
    .d_kqfilter =	rcrecv_kqfilter,
    .d_name =		RCRECV_CDEV_NAME,
};

/* 
 * Create sysctl variables and set their handlers
 */
static void
rcrecv_sysctl_register(struct rcrecv_softc *sc)
{
    struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(sc->dev);
    struct sysctl_oid *tree_node = device_get_sysctl_tree(sc->dev);
    struct sysctl_oid_list *tree = SYSCTL_CHILDREN(tree_node);

    SYSCTL_ADD_U32(ctx, tree, OID_AUTO, "value",
	CTLFLAG_RD,
	&sc->rc_code->value, 0, "Last received code");

    SYSCTL_ADD_S64(ctx, tree, OID_AUTO, "last_time",
	CTLFLAG_RD,
	&sc->rc_code->last_time, 0, "Last time when a code was received, us");

    SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "bit_length",
	CTLFLAG_RD,
	&sc->rc_code->bit_length, 0, "Received code length, bits");

    SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "proto",
	CTLFLAG_RD,
	&sc->rc_code->proto, 0, "Code transmission protocol");

    SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, "tolerance",
	CTLTYPE_U8 | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	&rcrecv_tolerance_sysctl, "CU", "Set tolerance for received signals, %");
}

#ifdef FDT

/*
 * Setup pin by FDT
 */
static int
rcrecv_fdt_setup_pin(struct rcrecv_softc *sc)
{
    int err;

    /* Try to configure our pin from fdt data on fdt-based systems. */
    err = gpio_pin_get_by_ofw_idx(sc->dev, ofw_bus_get_node(sc->dev), PIN_IDX,
	&sc->pin);

    return (err);
}

/*
 * Get FDT parameters
 */
static void
rcrecv_fdt_get_params(struct rcrecv_softc *sc)
{
    pcell_t param_cell;
    ssize_t param_found;
    uint32_t param;
    char * _from = "by default";

    param_found = OF_getencprop(ofw_bus_get_node(sc->dev), "default-tolerance", &param_cell, sizeof(param_cell));

    if (param_found > 0) {
	param = (uint32_t)param_cell;

	/* Check if greater than 100% */
	if (param > 100) {
	    device_printf(sc->dev,
			"Could not acquire correct tolerance percent %s\n", "from DTS");
	}
	else {
	    sc->receive_tolerance = (uint8_t)param;
	    _from = "from DTS";
	}
    }

    if (bootverbose)
	device_printf(sc->dev,
		"Acquired tolerance: %u%% %s\n", sc->receive_tolerance, _from);

    return;
}

#endif

/*
 * Setup pin by hints
 */
static int
rcrecv_hinted_setup_pin(struct rcrecv_softc *sc)
{
    const char *busname;
    int err;

    device_t busdev = device_get_parent(sc->dev);
    const char *devname = device_get_name(sc->dev);
    int unit = device_get_unit(sc->dev);

    /*
     * If there is not an "at" hint naming our actual parent, then we
     * weren't instantiated as a child of gpiobus via hints, and we thus
     * can't access ivars that only exist for such children.
     */
    if (resource_string_value(devname, unit, "at", &busname) != 0 ||
	(strcmp(busname, device_get_nameunit(busdev)) != 0 &&
	 strcmp(busname, device_get_name(busdev)) != 0))
    {
	return (ENOENT);
    }

    /* Get the pin number */
    err = gpio_pin_get_by_child_index(sc->dev, PIN_IDX, &sc->pin);

    /* If we didn't get configured by either method, whine and punt. */
    if (err != 0)
	device_printf(sc->dev,
	    "Cannot acquire gpio pin (config error)\n");

    return (err);
}

/*
 * Get hinted parameters
 */
static int
rcrecv_hinted_get_params(struct rcrecv_softc *sc)
{
    int err;
    uint32_t param;
    const char *devname = device_get_name(sc->dev);
    int unit = device_get_unit(sc->dev);
    char * _from = "by default";

    err = resource_int_value(devname, unit, "default-tolerance", &param);
    if (err == 0) {
	if (param > 100) {
	    device_printf(sc->dev,
			"Could not acquire correct tolerance percent %s\n", "from hints");
	    return (EINVAL);
	}
	else {
	    sc->receive_tolerance = (uint8_t)param;
	    _from = "from hints";
	}
    }

    if (bootverbose)
	device_printf(sc->dev,
		"Acquired tolerance: %u%% %s\n", sc->receive_tolerance, _from);

    return (0);
}

/*
 * Sysctl parameter: tolerance
 */
static int
rcrecv_tolerance_sysctl(SYSCTL_HANDLER_ARGS)
{
    struct rcrecv_softc *sc = arg1;
    uint8_t tolerance = sc->receive_tolerance;
    int err;

    err = SYSCTL_OUT(req, &tolerance, sizeof(tolerance));
    if (err != 0 || req->newptr == NULL)
	return (err);

    err = SYSCTL_IN(req, &tolerance, sizeof(tolerance));
    if (err != 0)
	return (err);

    if (tolerance > 100)
	return (EINVAL);

    sc->receive_tolerance = tolerance;

    return (0);
}

static inline unsigned int
diff(int A, int B)
{
    return abs(A - B);
}

static bool
rcrecv_decode_sequence(struct rcrecv_softc *sc, const size_t n)
{
    struct rcrecv_seq *seq = sc->rc_seq;
    const protocol *p = &(proto[n]);
    // Assuming the longer pulse length is the pulse captured in seq->timings[0]
    const size_t sync_length =  ((p->sync_factor.low) > (p->sync_factor.high)) ? (p->sync_factor.low) : (p->sync_factor.high);
    const size_t delay = seq->timings[0] / sync_length;
    const size_t delay_tolerance = delay * sc->receive_tolerance / 100;

    const size_t delay_zero_high = delay * p->zero.high;
    const size_t delay_zero_low  = delay * p->zero.low;
    const size_t delay_one_high  = delay * p->one.high;
    const size_t delay_one_low   = delay * p->one.low;

    const size_t first_timing = (p->inverted) ? 2 : 1;

    register unsigned long code = 0;
    for (register size_t i = first_timing; i < seq->edges_count - 1; i += 2)
    {
	code <<= 1;
	if (diff(seq->timings[i],   delay_zero_high) < delay_tolerance &&
	    diff(seq->timings[i+1], delay_zero_low)  < delay_tolerance) {
	    // zero
	}
	else
	if (diff(seq->timings[i],   delay_one_high) < delay_tolerance &&
	    diff(seq->timings[i+1], delay_one_low)  < delay_tolerance) {
	    // one
	    code |= 1;
	}
	else {
	    // Failed
	    return false; 
	}
    } // for (size_t i = 1; i < seq->edges_count - 1;

    /* ignore very short transmissions: no device sends them,
       so this must be noise
     */
    if (seq->edges_count > sc->edges_count_min) {
	struct rcrecv_code *rcc = sc->rc_code;

	rcc->last_time = sc->last_evtime;
	rcc->value = code;
	rcc->bit_length = (seq->edges_count - 1) / 2;
	rcc->proto = n + 1;
	rcc->ready = true;
	rcrecv_notify(sc);
    }

    return true;
}

static void
rcrecv_ihandler(void *arg)
{
    struct rcrecv_softc *sc = arg;
    struct rcrecv_seq *seq = sc->rc_seq;

    /* Capture time and pin state first. */
    const int64_t evtime = sbttous(sbinuptime());
    const size_t duration = (size_t)(evtime - sc->last_evtime);

    /* It is a start of a new sequence even if previous is unfinished */
    if (duration > SEPARATION_GAP_LIMIT)
    {
	/* When the sequence is already marked as completed and its first duration time
	   between edges (seq->timings[0]) is almost the same with this long one (duration),
	   it is time to try to transform the sequence into a code and clear the mark.
	   In another case the sequence must be marked as finished for worked it out on
	   a next interrupt by impulse edge.
	 */
	if (seq->completed && (diff(duration, seq->timings[0]) < SEPARATION_GAP_DELTA)) {
	    /* Try protocols one by one */
	    register size_t p = 0;
	    do {
		if (rcrecv_decode_sequence(sc, p)) break;
	    } while(++p < NELEMS(proto));
	    /* All done. Prepare to a new sequence */
	    seq->completed = false;
	}
	else
	    seq->completed = true;

	seq->edges_count = 0;
    }
    else if (seq->edges_count >= RCSWITCH_MAX_CHANGES) {
	/* If overflow occured */
	seq->completed = false;
	seq->edges_count = 0;
    }

    sc->last_evtime = evtime;

    /* Save an impulse duration to its either next or start position */
    seq->timings[seq->edges_count++] = duration;
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

    device_set_desc(dev, "GPIO Remote Control Receiver module");

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

    /* Destroy the rcrecv cdev. */
    if (sc->cdev != NULL) {
	mtx_lock(&sc->mtx);
	sc->cdev->si_drv1 = NULL;
	/* Wake everyone */
	rcrecv_notify(sc);
	mtx_unlock(&sc->mtx);
	destroy_dev(sc->cdev);
    }

    knlist_destroy(&sc->rsel.si_note);
    seldrain(&sc->rsel);
    free(sc->rc_seq, M_RCRECVSEQ);
    free(sc->rc_code, M_RCRECVCODE);

    return (0);
}

static int
rcrecv_attach(device_t dev)
{
    uint32_t pincaps, edge = GPIO_INTR_EDGE_BOTH;
    int err;

    struct rcrecv_softc *sc = device_get_softc(dev);

    sc->dev = dev;
    sc->rc_code = malloc(sizeof(*sc->rc_code), M_RCRECVCODE, M_WAITOK | M_ZERO);
    sc->rc_seq  = malloc(sizeof(*sc->rc_seq),  M_RCRECVSEQ,  M_WAITOK | M_ZERO);

    mtx_init(&sc->mtx, "rcrecv_mtx", NULL, MTX_DEF);
    knlist_init_mtx(&sc->rsel.si_note, &sc->mtx);

    rcrecv_sysctl_register(sc);

    sc->edges_count_min = 7;
    sc->poll_sel = true;
    sc->receive_tolerance = RECEIVE_TOLERANCE;

#ifdef FDT
    if ((err = rcrecv_fdt_setup_pin(sc)) == 0)
	rcrecv_fdt_get_params(sc);
#else
    err = ENOENT;
#endif

    /*
     * If we didn't get configured by fdt data and our parent is gpiobus,
     * see if we can be configured by the bus (allows hinted attachment even
     * on fdt-based systems).
     */
    if (err != 0) {
	if ((err = rcrecv_hinted_setup_pin(sc)) != 0) {
	    rcrecv_detach(dev);
	    return (err);
	}

	/* If there are any hinted parameters */
	rcrecv_hinted_get_params(sc);
    }

    /* Say what we came up with for pin config. */
    device_printf(dev, "Signal input on %s pin %u\n",
	device_get_nameunit(GPIO_GET_BUS(sc->pin->dev)), sc->pin->pin);

    /* Check if pin can be configured for interrupts */
    if ((err = gpio_pin_getcaps(sc->pin, &pincaps)) != 0) {
	device_printf(dev, "Cannot query capabilities of gpio pin\n");
	rcrecv_detach(dev);
	return (err);
    }
    if ((pincaps & edge) == 0) {
	device_printf(dev, "Pin cannot be configured for both signal edges interrupt\n");
	rcrecv_detach(dev);
	return (ENOTSUP);
    }

    /*
     * Create an interrupt resource from the pin and set up the interrupt.
     */
    if ((sc->intr_res = gpio_alloc_intr_resource(sc->pin->dev, &sc->intr_rid, RF_ACTIVE,
	sc->pin, edge)) == NULL) {
	    device_printf(dev, "Cannot allocate an IRQ for the GPIO\n");
	    rcrecv_detach(dev);
	    return (ENOTSUP);
    }

    err = bus_setup_intr(sc->pin->dev, sc->intr_res, INTR_TYPE_MISC | INTR_MPSAFE,
        NULL, rcrecv_ihandler, sc, &sc->intr_cookie);

    if (err != 0) {
	device_printf(dev, "Unable to setup RC receiver irq handler\n");
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
	device_printf(dev, "Unable to create RC receiver cdev\n");
	rcrecv_detach(dev);
	return (err);
    }

    sc->cdev->si_drv1 = sc;

    return (err);
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
    struct rcrecv_softc *sc = cdev->si_drv1;
    struct rcrecv_code *rcc = sc->rc_code;

    register uint32_t code;
    size_t len = 0;
    size_t amnt;
    char *dest;
    int err = 0;
    off_t uio_offset_saved;

    /* Exit normally but no realy uiomove() if not ready */
    if (!rcc->ready)
	return (err);

    code = rcc->value;
    len = rcc->bit_length;
    len >>= 2;
    if (rcc->bit_length & 0x3)
	len++;

    dest = sc->received_code + len;

    /* Fill the buffer from right to left */
    for (register size_t i = 0; i < len; i++) {
	/* Begin from len-1 */
	*--dest = '0' + (code & 0xf);
	if (*dest > '9')
	    *dest += OFFSET_A;
	code >>= 4;
    }

    mtx_lock(&sc->mtx);
    uio_offset_saved = uio->uio_offset;

    amnt = MIN(uio->uio_resid,
	      (len - uio->uio_offset > 0) ?
	       len - uio->uio_offset : 0);
    err = uiomove(sc->received_code, amnt, uio);

    uio->uio_offset = uio_offset_saved;
    mtx_unlock(&sc->mtx);

    if (err != 0)
	uprintf("uiomove failed!\n");
    else
	rcc->ready = false;

    return (err);
}

static int
rcrecv_ioctl(struct cdev *cdev, u_long cmd, caddr_t data, int fflag, struct thread *td)
{
    struct rcrecv_softc *sc = cdev->si_drv1;
    struct rcrecv_code *rcc = sc->rc_code;
    int err = 0;

    switch (cmd) {
    case RCRECV_READ_CODE:
	if (rcc->ready) {
	    rcc->ready = false;
	    *(uint32_t *)data = rcc->value;
	}
	else
	    data = NULL;
	break;
    case RCRECV_READ_CODE_INFO:
	rcc->ready = false;
	*(struct rcrecv_code *)data = *rcc;
	break;
    default:
#ifdef DEBUG
	uprintf("Undeclared ioctl(0x%lx)\n", cmd);
#endif
	err = ENOTTY;
	break;
    }

    return (err);
}

static int
rcrecv_poll(struct cdev *dev, int events, struct thread *td)
{
    int revents = 0;
    struct rcrecv_softc *sc = dev->si_drv1;

    if (sc == NULL)
	return (POLLHUP);

    if (events & (POLLIN | POLLRDNORM)) {
	mtx_lock(&sc->mtx);
	if (!sc->poll_sel) {
	    sc->poll_sel = true;
	    revents = events & (POLLIN | POLLRDNORM);
	}
	else
	    selrecord(td, &sc->rsel);
	mtx_unlock(&sc->mtx);
    }

    return (revents);
}

static void
rcrecv_notify(struct rcrecv_softc *sc)
{
    mtx_assert(&sc->mtx, MA_OWNED);

    if (sc->poll_sel) {
	sc->poll_sel = false;
	selwakeuppri(&sc->rsel, PZERO);
    }

    KNOTE_LOCKED(&sc->rsel.si_note, 0);
}

static int
rcrecv_kqfilter(struct cdev *dev, struct knote *kn)
{
    struct rcrecv_softc *sc = dev->si_drv1;

    if (sc == NULL)
	return (ENXIO);

    switch (kn->kn_filter) {
    case EVFILT_READ:
	kn->kn_fop = &rcrecv_filterops;
	kn->kn_hook = sc;
	mtx_lock(&sc->mtx);
	knlist_add(&sc->rsel.si_note, kn, 1);
	mtx_unlock(&sc->mtx);
	break;
    default:
	return (EINVAL);
	//return (EOPNOTSUPP);
    }

    return (0);
}

static int
rcrecv_kqevent(struct knote *kn, long hint)
{
    struct rcrecv_softc *sc = kn->kn_hook;
    struct rcrecv_code *rcc = sc->rc_code;
    size_t len;

    mtx_assert(&sc->mtx, MA_OWNED);

    if (rcc->ready) {
	len = rcc->bit_length;
	len >>= 2;
	if (rcc->bit_length & 0x3)
	    len++;

	kn->kn_data = len;
	return (1);
    }
    else
	return (0);
}

static void
rcrecv_kqdetach(struct knote *kn)
{
    struct rcrecv_softc *sc = kn->kn_hook;

    knlist_remove(&sc->rsel.si_note, kn, 0);
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
DRIVER_MODULE(rcrecv, simplebus, rcrecv_driver, rcrecv_devclass, 0, 0);
#endif

DRIVER_MODULE(rcrecv, gpiobus, rcrecv_driver, rcrecv_devclass, 0, 0);
MODULE_VERSION(rcrecv, 1);
MODULE_DEPEND(rcrecv, gpiobus, 1, 1, 1);
