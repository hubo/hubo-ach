#
# Makefile for the kernel serial device drivers.
#
# Advantech eAutomation Division
#


UNAME := $(shell uname -r)
PWD := $(shell pwd)
ADVMOD := adv950


SERIAL_NAME= ttyAP
MAX_TTY= 64 
ADV_TTY_MAJOR= 30
 
SERIAL_FLAGS= -DSERIAL_NAME=\"$(SERIAL_NAME)\" \
              -DADV_TTY_MAJOR=$(ADV_TTY_MAJOR) \
              -DADV_TTY_CUMAJOR=$(ADV_TTY_CUMAJOR)       
	      

ifeq "$(UNAME)""2.6.15-26-386" 
SERIAL_FLAGS:= $(SERIAL_FLAGS) -DUBUNTU_2_6_15=1
else  
ifeq "$(UNAME)""2.6.9-34.EL" 
SERIAL_FLAGS:= $(SERIAL_FLAGS) -DRHE4SMP=0
else 
ifeq "$(UNAME)""2.6.9-34.ELsmp" 
SERIAL_FLAGS:= $(SERIAL_FLAGS) -DRHE4SMP=1
else
SERIAL_FLAGS:= $(SERIAL_FLAGS) -DUBUNTU_2_6_15=0
endif
endif
endif

ifneq ($(KERNELRELEASE),)
EXTRA_CFLAGS  = $(SERIAL_FLAGS)
obj-m := adv950.o
adv950-objs := 8250.o 8250_pci.o serial_core.o  
else
	KDIR := /lib/modules/$(shell uname -r)/build/
	PWD  := $(shell pwd)
all:	
	$(MAKE) -w -C $(KDIR) SUBDIRS=$(PWD) modules
endif


install:
	$(shell if grep $(ADVMOD) /proc/modules > /dev/null ; then \
	 rmmod $(ADVMOD) ; fi)
	@insmod $(ADVMOD).ko

node: default install
	@echo "Creating serial device nodes $(SERIAL_NAME) ..."
	@chmod 777 ../advmknod
	@../advmknod $(SERIAL_NAME) \
	 $$(awk '$$2=="$(SERIAL_NAME)" {print $$1}' /proc/devices) $(MAX_TTY)
	@echo "Done"

uninstall:
	$(shell if grep $(ADVMOD) /proc/modules > /dev/null ; then \
	 rmmod $(ADVMOD) ; fi)

test:
	@cd ../getconfig; make; ./getconfig $(SERIAL_NAME) $(MAX_TTY)

clean:
	@rm -f *.o
	@rm -f *.ko
	@rm -f *.mod.c modules.order  Module.symvers Module.markers
	@rm -f .*.cmd
	@rm -rf .tmp_versions
	@rm -rf *~
	@cd ../getconfig; make clean
