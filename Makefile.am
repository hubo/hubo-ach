ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include

#EXTRA_DIST = hubo-ach-safe-clean.sh virtualHubo.py joint.table sensor.table scripts/hubo-ach drc-hubo-beta-1-default.home drc-hubo-beta-2-default.home drc-hubo-beta-2-rainbow-default.home
EXTRA_DIST = tables/drc-hubo-beta-1-default.home tables/drc-hubo-beta-2-default.home tables/drc-hubo-beta-2-rainbow-default.home tables/huboplus.joint.table tables/drc-hubo.joint.table tables/joint.table tables/sensor.table tables/drc-hubo.sensor.table tables/huboplus.sensor.table python/virtualHubo.py hubo-ach-safe-clean.sh python/hubo_drc_head.py python/settings.yaml


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

huboachconf_DATA = tables/drc-hubo-beta-1-default.home tables/drc-hubo-beta-2-default.home tables/drc-hubo-beta-2-rainbow-default.home tables/huboplus.joint.table tables/drc-hubo.joint.table tables/joint.table tables/sensor.table tables/drc-hubo.sensor.table tables/huboplus.sensor.table python/virtualHubo.py hubo-ach-safe-clean.sh python/hubo_drc_head.py python/settings.yaml

#hubo_jointparamsdir = $(prefix)/lib
hubo_jointparamsdir = /usr/lib
hubo_jointparams_LTLIBRARIES = libhuboparams.la
libhuboparams_la_SOURCES = src/hubo-jointparams.c
libhuboparams_la_LDFLAGS = -module -avoid-version -shared

bin_PROGRAMS = hubo-daemon hubo-console hubo-read hubo-homeparams hubo-log

hubo_daemon_SOURCES = src/hubo-daemonizer.c src/hubo-daemon.c src/hubo-socketcan.c
hubo_daemon_LDADD = libhuboparams.la -lm -lrt -lc

hubo_console_SOURCES = src/hubo-console.cpp
hubo_console_LDADD = -lrt -lc -lm -lreadline libhuboparams.la -lach

hubo_read_SOURCES = src/hubo-read.c
hubo_read_LDADD = libhuboparams.la -lm -lc -lrt -lach

hubo_homeparams_SOURCES = src/hubo-homeparams.cpp
hubo_homeparams_LDADD = libhuboparams.la -lm -lc -lrt -lach

hubo_log_SOURCES = src/hubo-log.c
hubo_log_LDADD = libhuboparams.la -lm -lc -lrt -lach

python_PYTHON = python/hubo_ach.py

