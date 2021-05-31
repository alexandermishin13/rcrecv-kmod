.PHONY: clean

all: ioctl_magic kevent_test getopt

clean:
	rm ioctl_magic
	rm kevent_test
	rm getopt

ioctl_magic: $@.c
	$(CC) $< -o $@

kevent_test: $@.c
	$(CC) $< -lgpio -lutil -o $@
