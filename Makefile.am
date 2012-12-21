ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include

EXTRA_DIST = jointtab scripts/hubo-ach

init_ddir = $(sysconfdir)/init.d
init_d_SCRIPTS = scripts/hubo-ach

# installed under $prefix/include/
include_HEADERS = include/hubo.h

# installed under $prefix/include/hubo-ach/
pkginclude_HEADERS = 	                \
	include/hubo/hubo-esdcan.h      \
	include/hubo/hubo-socketcan.h   \
	include/hubo/canId.h

# not installed
noinst_HEADERS =                        \
	include/hubo-daemon.h           \
	include/hubo-ref-filter.h       \
	include/hubo-jointparams.h

huboachconfdir=$(sysconfdir)/hubo-ach

huboachconf_DATA = jointtab

bin_PROGRAMS = hubo-daemon hubo-console hubo-loop hubo-read hubo-ref-filter

hubo_daemon_SOURCES = src/hubo-daemonizer.c src/hubo-daemon.c src/hubo-jointparams.c src/hubo-socketcan.c
hubo_daemon_LDADD = -lm

hubo_console_SOURCES = src/hubo-jointparams.c src/hubo-console.cpp
hubo_console_LDADD = -lreadline

hubo_loop_SOURCES = src/hubo-jointparams.c src/hubo-loop.c
hubo_loop_LDADD = -lm

hubo_read_SOURCES = src/hubo-jointparams.c src/hubo-read.c

hubo_ref_filter_SOURCES = src/hubo-jointparams.c src/hubo-ref-filter.c
