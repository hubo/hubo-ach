#
#
# Advcan -- LINUX device driver Makefile
#
# Copyright (c) 2004-2010 advantech
#

TARGET_MATCHED = false

CAN_MAJOR  :=  $(shell awk '$$2=="advcan" {print $$1}' /proc/devices)

DEFS = -DCAN_SYSCLK=8 -DADVANTECH

ifneq ($(KERNELRELEASE),)
EXTRA_CFLAGS  = $(DEFS) -DVERSION=\"advcan-2.09\"
obj-m := advcan.o
advcan-objs	:= core.o	\
	    open.o		\
	    read.o		\
	    write.o		\
	    ioctl.o		\
	    select.o	\
	    debug.o		\
	    error.o		\
	    util.o		\
	    sysctl.o	\
	    async.o		\
	    close.o		\
	    sja1000funcs.o	\
		 adv_pci.o   \
       adv_isa.o   
else
	KDIR := /lib/modules/$(shell uname -r)/build/
	PWD  := $(shell pwd)
all:
	@echo "This driver support:PCI-1680,PCI-1682U,MIC-3680,UNO2052,UNO2052(E),PCM-3680"
	$(MAKE) -w -C $(KDIR) SUBDIRS=$(PWD) modules
endif


# unload the driver module
uninstall:
	@echo "Removing Driver Module from Kernel"
	$(shell if grep advcan /proc/modules > /dev/null ; then \
	 rmmod advcan; fi)

clean:
	-@rm -f tags
	-@rm -f *.o *.ko
	-@rm -f .*.cmd *.mod.c
	-@rm -rf .tmp_versions
	-@rm -rf *~
	-@rm -rf *odule*

install:
	$(shell ! [ -d /lib/modules/$(shell uname -r)/kernel/drivers/advcan ] && mkdir /lib/modules/$(shell uname -r)/kernel/drivers/advcan)
	@echo "Insert driver to kernel"
	$(shell [ -f advcan.ko ]  && cp ./advcan.ko /lib/modules/$(shell uname -r)/kernel/drivers/advcan/  && depmod && modprobe advcan)

nodes:
	@echo "Creating devices file for Advantech CAN driver..."
	@rm -rf /dev/can*
	-mknod /dev/can0 c $(CAN_MAJOR) 0
	-mknod /dev/can1 c $(CAN_MAJOR) 1
	-mknod /dev/can2 c $(CAN_MAJOR) 2
	-mknod /dev/can3 c $(CAN_MAJOR) 3
	-mknod /dev/can4 c $(CAN_MAJOR) 4
	-mknod /dev/can5 c $(CAN_MAJOR) 5
	-mknod /dev/can6 c $(CAN_MAJOR) 6
	-mknod /dev/can7 c $(CAN_MAJOR) 7
	chmod 666 /dev/can[0-7]


