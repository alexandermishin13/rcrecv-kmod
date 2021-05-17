#include <sys/event.h>
#include <err.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <stdbool.h>
#include <unistd.h>
#include <dev/rcrecv/rcrecv.h>

void
usage()
{
	fprintf(stderr, "usage: %s [-f ctldev] [-m method] [-s] [-n] "
	    "[-t timeout] pin intr-config [pin intr-config ...]\n\n",
	    getprogname());
	fprintf(stderr, "Possible options for method:\n\n");
}

int
main(int argc, char **argv)
{
    int ch;
    char *file = "/dev/rcrecv";

    struct timespec timeout;
    int waitms = 10000;
    timeout.tv_sec = waitms / 1000;
    timeout.tv_nsec = (waitms % 1000) * 1000 * 1000;

    struct kevent event;    /* Event monitored */
    struct kevent tevent;   /* Event triggered */
    int kq, fd, ret;
    struct rcrecv_code rcc;
    unsigned long code;

    while ((ch = getopt(argc, argv, "f:")) != -1) {
	switch (ch) {
	case 'f':
	    file = optarg;
	    break;
	default:
	    usage();
	    return EXIT_FAILURE;
	}
    }
    argv += optind;
    argc -= optind;

    fd = open(file, O_RDONLY);
    /* Create kqueue. */
    kq = kqueue();
    if (kq == -1)
        err(EXIT_FAILURE, "kqueue() failed");

    /* Initialize kevent structure. */
    EV_SET(&event, fd, EVFILT_READ, EV_ADD | EV_CLEAR, 0, 0, NULL);
    /* Attach event to the kqueue. */
    ret = kevent(kq, &event, 1, NULL, 0, NULL);
    if (ret == -1)
        err(EXIT_FAILURE, "kevent register");
    if (event.flags & EV_ERROR)
        errx(EXIT_FAILURE, "Event error: %s", strerror(event.data));

    for (;;) {
        /* Sleep until a code received */
        ret = kevent(kq, NULL, 0, &tevent, 1, &timeout);
        if (ret == -1) {
            err(EXIT_FAILURE, "kevent wait");
        } else if (ret > 0) {
            ioctl(fd, RCRECV_READ_CODE_INFO, &rcc);
            printf("Received code: %lx\n", rcc.value);
        }
    }
}
