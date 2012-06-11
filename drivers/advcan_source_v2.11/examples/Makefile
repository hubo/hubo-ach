# Make some example applications using advcan device driver

CC=gcc

CFLAGS = -I../can4linux -lrt

CTAGS =	ctags --c-types=dtvf
CTAGS =	elvtags -tsevl

all: ctest baud  acceptance receive-block transmit-block receive-nonblock transmit-nonblock \
   receive-select send-ioctl selfreception \
   singlefilter transmit-select \


clean:
		-@rm -f ctest acceptance baud  \
		transmit-block  \
		transmit-nonblock \
		send-ioctl \
		transmit-select \
		receive-block \
		receive-nonblock \
		receive-select \
		singlefilter \
		selfreception \
		*~


