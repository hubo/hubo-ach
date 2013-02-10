ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include

EXTRA_DIST = joint.table sensor.table scripts/hubo-ach

#huboscriptdir = $(sysconfdir)/bin
huboscriptdir = /usr/bin
#init_ddir = $(sysconfdir)/init.d
init_ddir = $(huboscriptdir)
init_d_SCRIPTS = scripts/hubo-ach 

# installed under $prefix/include/
include_HEADERS = include/hubo.h include/hubo-jointparams.h include/hubo-daemonID.h
#include_HEADERS = include/hubo.h

# installed under $prefix/include/hubo-ach/
#otherincludedir = $(includedir)/hubo
#otherinclude_HEADERS = 	                \
#	include/hubo/hubo-esdcan.h      \
#	include/hubo/hubo-socketcan.h   \
#	include/hubo/canID.h            

# not installed

huboachconfdir=/etc/hubo-ach

huboachconf_DATA = tables/joint.table tables/sensor.table

#hubo_jointparamsdir = $(prefix)/lib
hubo_jointparamsdir = /usr/lib
hubo_jointparams_LTLIBRARIES = libhuboparams.la
libhuboparams_la_SOURCES = src/hubo-jointparams.c
libhuboparams_la_LDFLAGS = -module -avoid-version -shared

bin_PROGRAMS = hubo-daemon hubo-console hubo-read

hubo_daemon_SOURCES = src/hubo-daemonizer.c src/hubo-daemon.c src/hubo-socketcan.c
hubo_daemon_LDADD = -lm -lrt -lc libhuboparams.la

hubo_console_SOURCES = src/hubo-console.cpp
hubo_console_LDADD = -lrt -lc -lm -lreadline libhuboparams.la

hubo_read_SOURCES = src/hubo-read.c
hubo_read_LDADD = -lm -lc -lrt libhuboparams.la

