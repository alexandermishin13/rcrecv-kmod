.PHONY: clean

all: ioctl_magic kevent_test

clean:
	rm ioctl_magic
	rm kevent_test

ioctl_magic: $@.c
	$(CC) $< -o $@

kevent_test: $@.c
	$(CC) $< -lgpio -lutil -o $@
