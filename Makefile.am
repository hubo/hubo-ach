ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include

include_HEADERS = include/hubo.h
pkginclude_HEADERS = 	                \
	include/hubo/hubo-esdcan.h      \
	include/hubo/hubo-socketcan.h   \
	include/hubo/canId.h

bin_PROGRAMS = hubo-main hubo-default

hubo_main_SOURCES = src/hubo-main.c src/hubo-socketcan.c

hubo_default_SOURCES = src/hubo-default.c
