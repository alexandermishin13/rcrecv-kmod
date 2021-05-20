#include <libutil.h>
#include <sys/event.h>
#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <stdbool.h>
#include <unistd.h>
#include <getopt.h>
#include <signal.h>

#include <dev/rcrecv/rcrecv.h>

int fd;
struct pidfh *pfh;
bool background = false;

typedef struct rcc_entry {
    unsigned long code;
    unsigned int pin;
    bool state;
    SLIST_ENTRY(rcc_entry) switches;
} *rcc_entry_t;

struct rcc_list search_switch;

SLIST_HEAD(rcc_list, rcc_entry);

static void
usage()
{
    fprintf(stderr, "usage: %s [-d ctldev] -(s|u) <code>:pin [-b] [-h]"
	"\n\n",
	getprogname());
    fprintf(stderr, "Possible options for method:\n\n");
}

int i = 0;

/* Signals handler. Prepare the programm for end */
static void
termination_handler(int signum)
{
    rcc_entry_t node;

    /* Free allocated memory blocks */
    while (!SLIST_EMPTY(&search_switch)) {           /* List Deletion. */
	node = SLIST_FIRST(&search_switch);
	SLIST_REMOVE_HEAD(&search_switch, switches);
	free(node);
    }

    /* Close the device */
    close(fd);

    /* Remove pidfile and exit */
    pidfile_remove(pfh);

    exit(EXIT_SUCCESS);
}

/* Daemonize the program */
static void
daemonize(void)
{
    pid_t otherpid;

    /* Try to create a pidfile */
    pfh = pidfile_open(NULL, 0600, &otherpid);
    if (pfh == NULL)
    {
	if (errno == EEXIST)
	    errx(EXIT_FAILURE, "Daemon already running, pid: %jd.", (intmax_t)otherpid);

	/* If we cannot create pidfile from other reasons, only warn. */
	warn("Cannot open or create pidfile");
	/*
	 * Even though pfh is NULL we can continue, as the other pidfile_*
	 * function can handle such situation by doing nothing except setting
	 * errno to EDOOFUS.
	 */
    }

    /* Try to demonize the process */
    if (daemon(0, 0) == -1)
    {
	pidfile_remove(pfh);
	errx(EXIT_FAILURE, "Cannot daemonize");
    }

    pidfile_write(pfh);
}

/* Adds new code map entry */
static rcc_entry_t
search_rcc_entry(const unsigned long *code)
{
    rcc_entry_t node, tmpnode = NULL;

    SLIST_FOREACH(node, &search_switch, switches)
    {
	printf("code: %lu, pin: %u, state: %u\n", node->code, node->pin, node->state);
	if (node->code == *code) {
	    tmpnode = node;
	    break;
	}
    }

    return node;
}

/* Adds new code map entry */
static rcc_entry_t
add_rcc_entry(rcc_entry_t curnode)
{
    rcc_entry_t newnode;

    newnode = (rcc_entry_t)malloc(sizeof(*newnode));
    if (newnode == NULL)
	err(EXIT_FAILURE, "Unable to malloc symbol_node\n");

    if (curnode == NULL)
	SLIST_INSERT_HEAD(&search_switch, newnode, switches);
    else if (SLIST_NEXT(curnode, switches) == NULL)
	SLIST_INSERT_AFTER(curnode, newnode, switches);
    else
	err(EXIT_FAILURE, "Unable to add a switch entry\n");

    return newnode;
}

int
main(int argc, char **argv)
{
    rcc_entry_t node;
    int opt;
    char *dev_file = "/dev/rcrecv";
    char *end, *tmp;

    struct timespec timeout;
    int waitms = 10000;

    struct kevent event;    /* Event monitored */
    struct kevent tevent;   /* Event triggered */
    int kq, ret;
    struct rcrecv_code rcc;
    unsigned long code;

    int long_index =0;
    static struct option long_options[] = {
//        {"status", no_argument,       0, 'v' },
        {"device", required_argument, 0, 'd' },
        {"set",    required_argument, 0, 's' },
        {"unset",  required_argument, 0, 'u' },
        {"help",   required_argument, 0, 'h' },
        {0, 0, 0, 0}
    };

    SLIST_INIT(&search_switch);
    node = SLIST_FIRST(&search_switch);

    while ((opt = getopt_long(argc, argv, "d:s:u:b",long_options,&long_index)) != -1) {
	switch (opt) {
	case 'd':
	    dev_file = optarg;
	    break;
	case 'b':
	    background = true;
	    break;
	case 's':
	    node = add_rcc_entry(node);

	    /* Get a code from the argument */
	    tmp = strtok(optarg, ":");
	    if ((tmp == NULL) || (tmp[0] == '-'))
		errx(EXIT_FAILURE, "Wrong RC code value '%s': -%c <code>:<pin>\n", tmp, opt);
	    node->code = strtoul(tmp, &end, 0);

	    /* Get a pin number from the argument */
	    tmp = strtok(NULL, ":");
	    if (tmp == NULL)
		errx(EXIT_FAILURE, "Wrong pin number '%s': -%c %lu:<pin>\n", tmp, opt, node->code);
	    node->pin = strtoul(tmp, &end, 0);

	    /* Set a pin state */
	    node->state = true;
	    break;
	case 'u':
	    node = add_rcc_entry(node);

	    /* Get a code from the argument */
	    tmp = strtok(optarg, ":");
	    if ((tmp == NULL) || (tmp[0] == '-'))
		errx(EXIT_FAILURE, "Wrong RC code value '%s': -%c <code>:<pin>\n", tmp, opt);
	    node->code = strtoul(tmp, &end, 0);

	    /* Get a pin number from the argument */
	    tmp = strtok(NULL, ":");
	    if (tmp == NULL)
		errx(EXIT_FAILURE, "Wrong pin number '%s': -%c %lu:<pin>\n", tmp, opt, node->code);
	    node->pin = strtoul(tmp, &end, 0);

	    /* Set a pin state */
	    node->state = false;
	    break;
	case 'h':
	    /* FALLTHROUGH */
	default:
	    usage();
	    return EXIT_FAILURE;
	}
    }
    argv += optind;
    argc -= optind;

    /* Set a timeout by 'waitms' value */
    timeout.tv_sec = waitms / 1000;
    timeout.tv_nsec = (waitms % 1000) * 1000 * 1000;

    fd = open(dev_file, O_RDONLY);
    if (fd < 0) {
	perror("opening RC receiver device");
	exit(EXIT_FAILURE);
    }

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

    /* Unbinds from terminal if '-b' */
    if (background)
	daemonize();

    /* Intercept signals to our function */
    if (signal (SIGINT, termination_handler) == SIG_IGN)
	signal (SIGINT, SIG_IGN);
    if (signal (SIGTERM, termination_handler) == SIG_IGN)
	signal (SIGTERM, SIG_IGN);

    for (;;) {
	/* Sleep until a code received */
	ret = kevent(kq, NULL, 0, &tevent, 1, &timeout);
	if (ret == -1) {
	    err(EXIT_FAILURE, "kevent wait");
	} else if (ret > 0) {
	    ioctl(fd, RCRECV_READ_CODE_INFO, &rcc);
	    printf("Received code: %lx\n", rcc.value);
	    node = search_rcc_entry(&rcc.value);
	    if (node != NULL) {
		printf("Received code: %lx\n", rcc.value);
	    }
	}
    }

    exit(EXIT_SUCCESS);
}
