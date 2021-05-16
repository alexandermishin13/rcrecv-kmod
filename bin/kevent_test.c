#include <sys/event.h>
#include <err.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <dev/rcrecv/rcrecv.h>

int
main(int argc, char **argv)
{
    struct timespec timeout;
    int waitms = 10000;
    timeout.tv_sec = waitms / 1000;
    timeout.tv_nsec = (waitms % 1000) * 1000 * 1000;
//    timeout.tv_sec = 0;
//    timeout.tv_nsec = 0;
    struct kevent event;    /* Event we want to monitor */
    struct kevent tevent;   /* Event triggered */
    int kq, fd, ret;
    unsigned long code;

    if (argc != 2)
        err(EXIT_FAILURE, "Usage: %s path\n", argv[0]);
    fd = open(argv[1], O_RDONLY);
    if (fd == -1)
        err(EXIT_FAILURE, "Failed to open '%s'", argv[1]);

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
        /* Sleep until something happens. */
        ret = kevent(kq, NULL, 0, &tevent, 1, &timeout);
        if (ret == -1) {
            err(EXIT_FAILURE, "kevent wait");
        } else if (ret > 0) {
            ioctl(fd, RCRECV_READ_CODE, &code);
            printf("Received code: %lx\n", code);
        }
    }
}
