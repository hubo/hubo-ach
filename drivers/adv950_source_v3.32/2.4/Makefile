#
# Makefile for the PCI-954/16C950 serial drivers
#
# Advantech eAutomation Division
#

ADVMOD := adv950
UNAME := $(shell uname -r)
ifeq "$(UNAME)""2.4.27-2-386" 
LINUX_SRC= /usr/src/kernel-headers-2.4.27-2-386
else 
LINUX_SRC= /usr/src/linux-2.4
endif

CONFIG_PATH = config-$(shell uname -r)
RESULT = $(shell grep CONFIG_DEVFS_FS=y /boot/$(CONFIG_PATH))

ifeq ($(RESULT),CONFIG_DEVFS_FS=y)
SERIAL_NAME = ttap
SERIAL_FOLDER = ttap/%d
CALLOUT_NAME= ttac
CALLOUT_FOLDER = ttac/%d
TEST_NAME = ttap/
else
SERIAL_NAME = ttyAP
SERIAL_FOLDER = ttyAP
CALLOUT_NAME= ttyAC
CALLOUT_FOLDER = ttyAC
TEST_NAME = ttyAP
endif

MAX_TTY= 16
MAX_TTY= 16
ADV_TTY_MAJOR= 30
ADV_TTY_CUMAJOR= 35

ARCH_FLAGS= $(I386_FLAGS)
INCL_FLAGS= -I. -I$(LINUX_SRC)/include -I$(LINUX_SRC)/drivers/char
DEF_FLAGS= -D__KERNEL__ -DLINUX \
           -DCONFIG_PCI -DCONFIG_SERIAL_MULTIPORT \
           -DEXPORT_SYMTAB 
I386_FLAGS= -march=i686 -O2 -Wall
SERIAL_FLAGS= -DSERIAL_NAME=\"$(SERIAL_NAME)\" \
              -DCALLOUT_NAME=\"$(CALLOUT_NAME)\" \
              -DADV_TTY_MAJOR=$(ADV_TTY_MAJOR) \
              -DADV_TTY_CUMAJOR=$(ADV_TTY_CUMAJOR)
CFLAGS=	$(ARCH_FLAGS) $(INCL_FLAGS) $(DEF_FLAGS) $(SERIAL_FLAGS) -DMODULE
CC= gcc

default: $(ADVMOD).o 

$(ADVMOD).o: serial.c
	$(CC) $(CFLAGS) -c serial.c -o $(ADVMOD).o

install: default
	$(shell if grep $(ADVMOD) /proc/modules > /dev/null ; then \
         rmmod $(ADVMOD) ; fi)
	@insmod $(ADVMOD).o

node: default install
	@echo "Creating serial device nodes $(SERIAL_NAME) ..."
	@echo "The Result is $(RESULT)"
	@../advmknod $(SERIAL_NAME) \
	 $$(awk '$$2=="$(SERIAL_FOLDER)" {print $$1}' /proc/devices) $(MAX_TTY)
	@echo "Creating callout device nodes $(CALLOUT_NAME) ..."
	@../advmknod $(CALLOUT_NAME) \
	 $$(awk '$$2=="$(CALLOUT_FOLDER)" {print $$1}' /proc/devices) $(MAX_TTY)
	@echo "Done"

uninstall:
	$(shell if grep $(ADVMOD) /proc/modules > /dev/null ; then \
        rmmod $(ADVMOD) ; fi)

test:
	@cd ../getconfig; make; ./getconfig $(TEST_NAME) $(MAX_TTY)

clean:
	@rm -f $(ADVMOD).o
	@rm -f *~
	@cd ../getconfig; make clean
