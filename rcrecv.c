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

#include <sys/systm.h>  /* uprintf */
#include <sys/sysctl.h>
#include <sys/conf.h>   /* cdevsw struct */
#include <sys/param.h>  /* defines used in kernel.h */
#include <sys/module.h>
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

#define MEMBER_SIZE(type, member) \
    (sizeof(((type *)0)->member))

#define ELEMENTS_NUMBER(x) \
    (sizeof(x) / sizeof((x)[0]))

/* Use the first/only configured pin. */
#define	PIN_IDX 0

#define RCRECV_CDEV_NAME "rcrecv"

#define RCRECV_LOCK_INIT(sc)	\
    mtx_init(&(sc)->mtx, "rcrecv_mtx", NULL, MTX_DEF)
#define RCRECV_LOCK_DESTROY(sc)	\
    mtx_destroy(&(sc)->mtx)
#define RCRECV_LOCK(sc)		\
    mtx_lock(&(sc)->mtx)
#define RCRECV_UNLOCK(sc)	\
    mtx_unlock(&(sc)->mtx)

#define SEPARATION_GAP_LIMIT 4600
#define SEPARATION_GAP_DELTA 200
#define RECEIVE_TOLERANCE 60
/* Number of maximum rising/falling edges per packet.
 * We can handle up to (uint32_t) => 32 bit * 2 edges per bit + 2 edges for sync
 */
#define RCSWITCH_MAX_CHANGES 67
#define RCSWITCH_MIN_CHANGES 7

static const uint_fast8_t OFFSET_A = 'a' - '9' - 1;
static const uint_fast8_t PROTO_SIZE = ELEMENTS_NUMBER(proto);

MALLOC_DEFINE(M_RCRECVCODE, "rcrecvcode", "code info");
MALLOC_DEFINE(M_RCRECVSEQ, "rcrecvseq", "code edges timings");

struct rcrecv_seq {
    unsigned int	 timings[RCSWITCH_MAX_CHANGES];
    unsigned int	 edges_count;
    bool		 multiple;
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
    unsigned int	 edges_count_min;
    int			 intr_rid;
    struct selinfo	 rsel;
    struct mtx		 mtx;
    uint8_t		 receive_tolerance;
    bool		 poll_sel;
    char		 received_code[MEMBER_SIZE(struct rcrecv_code, value) * 2 + 1]; // two chars per byte + '\0'
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

    SYSCTL_ADD_S64(ctx, tree, OID_AUTO, "last_time",
	CTLFLAG_RD,
	&sc->rc_code->last_time, 0, "Last time when a code was received, us");

    SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "value",
	CTLFLAG_RD,
	&sc->rc_code->value, 0, "Last received code");

    SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "bit_length",
	CTLFLAG_RD,
	&sc->rc_code->bit_length, 0, "Received code length, bits");

    SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "proto",
	CTLFLAG_RD,
	&sc->rc_code->proto, 0, "Code transmission protocol");

    SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "pulse_duration",
	CTLFLAG_RD,
	&sc->rc_code->pulse_duration, 0, "Single pulse duration, usec");

    SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, "tolerance",
	CTLTYPE_U8 | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	&rcrecv_tolerance_sysctl, "CU", "Set tolerance for received signals, %");
}

#ifdef FDT

/*
 * Get FDT parameters
 */
static int
rcrecv_fdt_get_params(struct rcrecv_softc *sc)
{
    phandle_t node;
    pcell_t param_cell;
    ssize_t param_found;
    uint32_t param;
    int err;
    char * _from = "by default";

    /* Try to configure our pin from fdt data on fdt-based systems. */
    node = ofw_bus_get_node(sc->dev);
    if ((err = gpio_pin_get_by_ofw_idx(sc->dev, node, PIN_IDX, &sc->pin)) != 0)
	return (err);

    param_found = OF_getencprop(node, "default-tolerance", &param_cell, sizeof(param_cell));

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

    return (err);
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

static inline int
diff(int A, int B)
{
    return abs(A - B);
}

static bool
rcrecv_decode_sequence(struct rcrecv_softc *sc, const size_t n)
{
    struct rcrecv_code *rcc;
    struct rcrecv_seq *seq = sc->rc_seq;

    const protocol *p = &(proto[n]);
    // Assuming the longer pulse length is the pulse captured in seq->timings[0]
    const uint_fast8_t sync_length =  (p->sync_factor.low > p->sync_factor.high) ? p->sync_factor.low : p->sync_factor.high;
    const uint_fast32_t delay = (uint_fast32_t) seq->timings[0] / sync_length;
    const uint_fast32_t delay_tolerance = delay * sc->receive_tolerance / 100;

    const unsigned int delay_zero_high = delay * p->zero.high;
    const unsigned int delay_zero_low  = delay * p->zero.low;
    const unsigned int delay_one_high  = delay * p->one.high;
    const unsigned int delay_one_low   = delay * p->one.low;

    const uint_fast8_t first_timing_index = (p->inverted) ? 2 : 1;
    const uint_fast8_t edges_count = seq->edges_count - 1;

    uint_fast32_t code = 0;
    for (uint_fast8_t i = first_timing_index; i < edges_count; i++)
    {
	const unsigned int timing = seq->timings[i++]; // Second index increment
	code <<= 1;

	if (diff(timing, delay_zero_high) < delay_tolerance &&
	    diff(seq->timings[i], delay_zero_low) < delay_tolerance) {
	    // zero
	}
	else
	if (diff(timing, delay_one_high) < delay_tolerance &&
	    diff(seq->timings[i], delay_one_low) < delay_tolerance) {
	    // one
	    code |= 1;
	}
	else {
	    // Failed
	    return false; 
	}
    } // for (size_t i = 1; i < seq->edges_count - 1;

    /* All done. Save the variables and prepare to a new sequence */
    seq->edges_count = 0;

    rcc = sc->rc_code;

    rcc->last_time = sc->last_evtime;
    rcc->value = code;
    rcc->bit_length = edges_count / 2;
    rcc->pulse_duration = delay;
    rcc->proto = n + 1;
    rcc->ready = true;

    return true;
}

static int
rcrecv_ifltr(void *arg)
{
    /* Capture time and pin state first. */
    const int_fast64_t evtime = sbttous(sbinuptime());

    struct rcrecv_softc *sc = arg;
    struct rcrecv_seq *seq = sc->rc_seq;

    /* I belive that integer is too big for the air events period */
    const unsigned int duration = evtime - sc->last_evtime;
    sc->last_evtime = (int64_t)evtime;

    /* It is a start of a new sequence even if previous is unfinished */
    if (duration > SEPARATION_GAP_LIMIT) {
	/* When there are multiple code sequences and the last sequence's first duration time
	   between edges (seq->timings[0]) is almost the same with this long one (duration),
	   it is time to try to transform last sequence into a code and finish this round.
	 */
	if (seq->edges_count <= sc->edges_count_min) {
	    /* The sequence is too short, this round failed */
	    seq->multiple = false;
	}
	else
	if (seq->multiple && (diff(duration, seq->timings[0]) < SEPARATION_GAP_DELTA)) {
	    /* The round is over. Call the threaded handler to find the code */
	    seq->multiple = false;

	    return (FILTER_SCHEDULE_THREAD); 
	}
	else {
	    /* It wasn't really multiple sending of the code. Not now, but it still can be so.
	       Let's try the next sequence, we've already got a duration as it's timing[0].
	     */
	    seq->multiple = true;
	}
	seq->edges_count = 0;
    }
    else
    if (seq->edges_count >= RCSWITCH_MAX_CHANGES) {
    /* Check for overflow, if this isn't a start of new sequence */
	seq->multiple = false;
	seq->edges_count = 0;
    }

    /* Save an impulse duration to its either next or start position */
    seq->timings[seq->edges_count++] = duration;

    /* No needs for a threaded handler */
    return (FILTER_HANDLED);
}

static void
rcrecv_ithrd(void *arg)
{
    struct rcrecv_softc *sc = arg;

    /* Try protocols one by one */
    uint_fast8_t p = 0;
    do {
	if (rcrecv_decode_sequence(sc, p)) {
	    rcrecv_notify(sc);
	    break;
	}
    } while(++p < PROTO_SIZE);
}

/* Device _probe() method */
static int
rcrecv_probe(device_t dev)
{
    int rc;

    /*
     * By default we only bid to attach if specifically added by our parent
     * (usually via hint.rcrecv.#.at=busname).  On FDT systems we bid as
     * the default driver based on being configured in the FDT data.
     */
#ifdef FDT
    if (!ofw_bus_status_okay(dev))
	return (ENXIO);

    if (ofw_bus_search_compatible(dev, compat_data)->ocd_data)
	rc = BUS_PROBE_DEFAULT;
    else
#endif
	rc = BUS_PROBE_NOWILDCARD;

    device_set_desc(dev, "GPIO Remote Control Receiver module");

    return (rc);
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
	RCRECV_LOCK(sc);
	sc->cdev->si_drv1 = NULL;
	/* Wake everyone */
	rcrecv_notify(sc);
	RCRECV_UNLOCK(sc);
	destroy_dev(sc->cdev);
    }

    knlist_clear(&sc->rsel.si_note, 0);
    knlist_destroy(&sc->rsel.si_note);
    seldrain(&sc->rsel);
    free(sc->rc_seq, M_RCRECVSEQ);
    free(sc->rc_code, M_RCRECVCODE);

    RCRECV_LOCK_DESTROY(sc);

    return (0);
}

static int
rcrecv_attach(device_t dev)
{
    uint32_t pincaps, edge = GPIO_INTR_EDGE_BOTH;
    int err;

    struct rcrecv_softc *sc = device_get_softc(dev);

    sc->dev = dev;
    sc->rc_code = malloc(sizeof(struct rcrecv_code), M_RCRECVCODE, M_WAITOK | M_ZERO);
    sc->rc_seq  = malloc(sizeof(struct rcrecv_seq),  M_RCRECVSEQ,  M_WAITOK | M_ZERO);

    RCRECV_LOCK_INIT(sc);
    knlist_init_mtx(&sc->rsel.si_note, &sc->mtx);

    rcrecv_sysctl_register(sc);

    sc->edges_count_min = RCSWITCH_MIN_CHANGES;
    sc->poll_sel = true;
    sc->receive_tolerance = RECEIVE_TOLERANCE;

#ifdef FDT

    /*
     * If we didn't get configured by fdt data and our parent is gpiobus,
     * see if we can be configured by the bus (allows hinted attachment even
     * on fdt-based systems).
     */
    if ((err = rcrecv_fdt_get_params(sc)) != 0) {

#endif

	if ((err = rcrecv_hinted_setup_pin(sc)) != 0) {
	    rcrecv_detach(dev); // It was the last try to configure pin, so...
	    return (err);
	}

	/* If there are any hinted parameters */
	rcrecv_hinted_get_params(sc);

#ifdef FDT

    } // if ((err = rcrecv_fdt_get_params(sc))...

#endif

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
        rcrecv_ifltr, rcrecv_ithrd, sc, &sc->intr_cookie);

    if (err != 0) {
	device_printf(dev, "Unable to setup RC receiver irq handler\n");
	rcrecv_detach(dev);
	return (err);
    }

    /* Create the rcrecv cdev */
    err = make_dev_p(MAKEDEV_CHECKNAME | MAKEDEV_WAITOK,
	&sc->cdev,
	&rcrecv_cdevsw,
	0,
	UID_ROOT,
	GID_WHEEL,
	0600,
	RCRECV_CDEV_NAME
//	RCRECV_CDEV_NAME "%d",
//	device_get_unit(dev)
    );

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

    struct rcrecv_softc *sc = cdev->si_drv1;

    /* We can't be unloaded while open, so mark ourselves BUSY. */
    RCRECV_LOCK(sc);
    if (device_get_state(sc->dev) < DS_BUSY) {
	device_busy(sc->dev);
    }
    RCRECV_UNLOCK(sc);

#ifdef DEBUG
    uprintf("Device \"%s\" opened.\n", rcrecv_cdevsw.d_name);
#endif

    return (0);
}

static int
rcrecv_close(struct cdev *cdev __unused, int fflag __unused, int devtype __unused,
    struct thread *td __unused)
{
    struct rcrecv_softc *sc = cdev->si_drv1;

    /*
     * Un-busy on last close. We rely on the vfs counting stuff to only call
     * this routine on last-close, so we don't need any open-count logic.
     */
    RCRECV_LOCK(sc);
    device_unbusy(sc->dev);
    RCRECV_UNLOCK(sc);

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

    uint_fast32_t code;
    char *dest;
    int amnt;
    int err = 0;
    off_t uio_offset_saved;
    uint_fast8_t len = 0;

    /* Exit normally but no realy uiomove() if not ready */
    if (!rcc->ready)
	return (err);

    code = rcc->value;
    /* the number of nibbles for the length in bits */
    len = (rcc->bit_length + 3) / 4;
    /* Start backward from the last one */
    dest = sc->received_code + len;

    /* Fill the buffer from right to left */
    for (uint_fast8_t i = 0; i < len; i++) {
	/* One character back */
	*--dest = '0' + ((char)code & 0x0f);
	if (*dest > '9')
	    *dest += OFFSET_A;
	code /= 16;
    }

    RCRECV_LOCK(sc);
    uio_offset_saved = uio->uio_offset;

    amnt = MIN(uio->uio_resid,
	      (len - uio->uio_offset > 0) ?
	       len - uio->uio_offset : 0);
    err = uiomove(sc->received_code, amnt, uio);

    uio->uio_offset = uio_offset_saved;
    RCRECV_UNLOCK(sc);

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

    RCRECV_LOCK(sc);
    switch (cmd) {
    case RCRECV_READ_CODE:
	if (rcc->ready) {
	    rcc->ready = false;
	    *(unsigned int *)data = rcc->value;
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
    RCRECV_UNLOCK(sc);

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
	RCRECV_LOCK(sc);
	if (!sc->poll_sel) {
	    sc->poll_sel = true;
	    revents = events & (POLLIN | POLLRDNORM);
	}
	else
	    selrecord(td, &sc->rsel);
	RCRECV_UNLOCK(sc);
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

    /* Only ENVFILT_READ is used */
    if (kn->kn_filter != EVFILT_READ)
	return (EINVAL);

    kn->kn_fop = &rcrecv_filterops;
    kn->kn_hook = sc;
    knlist_add(&sc->rsel.si_note, kn, 0);

    return (0);
}

static int
rcrecv_kqevent(struct knote *kn, long hint)
{
    struct rcrecv_softc *sc = kn->kn_hook;
    struct rcrecv_code *rcc = sc->rc_code;
    uint_fast8_t len;

    mtx_assert(&sc->mtx, MA_OWNED);

    if (!rcc->ready)
	return (0);

    /* the number of nibbles for the length in bits */
    len = (rcc->bit_length + 3) / 4;
    kn->kn_data = len;

    return (1);
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
