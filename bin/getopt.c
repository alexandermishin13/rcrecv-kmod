#include <unistd.h>
#include <stddef.h>
#include <limits.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <err.h>
#include <sys/queue.h>

#include <stdio.h>

typedef struct rcc_entry {
    unsigned long code;
    unsigned int pin;
    bool state;
    SLIST_ENTRY(rcc_entry) switches;
} *rcc_entry_t;

struct rcc_list search_switch;

SLIST_HEAD(rcc_list, rcc_entry);
// rcc_head = SLIST_HEAD_INITIALIZER(rcc_head);

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
main(int argc, char *argv[])
{
    rcc_entry_t node, tmpnode;
    char *end, *tmp;
    int opt;

    SLIST_INIT(&search_switch);
    node = SLIST_FIRST(&search_switch);

    while ((opt = getopt(argc, argv, "s:u:")) != -1) {
	switch (opt) {
	case 's':
	    node = add_rcc_entry(node);

	    /* Get a code from the argument */
	    tmp = strtok(optarg, ":");
	    if ((tmp == NULL) || (tmp[0] == '-'))
		errx(EXIT_FAILURE, "Wrong RC code value '%s': -%c <code>:<pin>\n", tmp, opt);
	    node->code = strtoul(tmp, &end, 10);

	    /* Get a pin number from the argument */
	    tmp = strtok(NULL, ":");
	    if (tmp == NULL)
		errx(EXIT_FAILURE, "Wrong pin number '%s': -%c %lu:<pin>\n", tmp, opt, node->code);
	    node->pin = strtoul(tmp, &end, 10);

	    /* Set a pin state */
	    node->state = true;
	    break;
	case 'u':
	    node = add_rcc_entry(node);

	    /* Get a code from the argument */
	    tmp = strtok(optarg, ":");
	    if ((tmp == NULL) || (tmp[0] == '-'))
		errx(EXIT_FAILURE, "Wrong RC code value '%s': -%c <code>:<pin>\n", tmp, opt);
	    node->code = strtoul(tmp, &end, 10);

	    /* Get a pin number from the argument */
	    tmp = strtok(NULL, ":");
	    if (tmp == NULL)
		errx(EXIT_FAILURE, "Wrong pin number '%s': -%c %lu:<pin>\n", tmp, opt, node->code);
	    node->pin = strtoul(tmp, &end, 10);

	    /* Set a pin state */
	    node->state = false;
	    break;
	}
    }
    argv += optind;
    argc -= optind;

    SLIST_FOREACH(node, &search_switch, switches)
    {
	printf("code: %lu, pin: %u, state: %u\n", node->code, node->pin, node->state);
    }

    SLIST_FOREACH_SAFE(node, &search_switch, switches, tmpnode)
    {
	SLIST_REMOVE(&search_switch, node, rcc_entry, switches);
	free(node);
    }

    return (0);
}
