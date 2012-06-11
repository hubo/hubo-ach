VERS26=$(findstring 2.6,$(shell uname -r))
DRVDIR?=$(shell pwd)
export DRVDIR
ifeq ($(VERS26),2.6)
include $(DRVDIR)/Makefile-2.6.inc
else
include $(DRVDIR)/Makefile-2.4.inc
endif
