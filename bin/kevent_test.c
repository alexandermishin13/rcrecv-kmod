#include <stdio.h>
#include <stdlib.h>
#include <err.h>
#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <sys/event.h>

extern char * __progname;

int main(int argc, char * argv[])
{

    int fd, kq;
    struct kevent ke;

    if ( argc < 2 )
	errx(1,"usage: %s file",__progname);

    if ( ( fd = open(argv[1], O_RDONLY, NULL) ) == NULL )
	err(1,"%s:%i",__FILE__,__LINE__);

    if ( ( kq = kqueue() ) == -1 )
	err(1,"%s:%i",__FILE__,__LINE__);

    bzero(&ke,sizeof(ke));

    EV_SET(&ke, fd, EVFILT_VNODE, EV_ADD|EV_CLEAR, NOTE_WRITE, 0, NULL);

    if ( kevent(kq, &ke, 1, NULL, 0, NULL) == -1 )
	err(1,"%s:%i",__FILE__,__LINE__);

    bzero(&ke,sizeof(ke));

    while(1)
	if ( kevent(kq, NULL, 0, &ke, 1, NULL) != -1 )
	    printf("A write occurred on the file.n");

    return 0;
}

