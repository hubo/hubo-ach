#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.67])
AC_INIT([hubo-ach], [0.0.0], [dml46@drexel.edu])
AM_INIT_AUTOMAKE([-Wall])
AC_CONFIG_SRCDIR([hubo.h])
AC_CONFIG_HEADERS([config.h])

# Checks for programs.
AC_USE_SYSTEM_EXTENSIONS
AC_PROG_CC
AC_PROG_LIBTOOL

# Checks for libraries.
# FIXME: Replace `main' with a function in `-lach':
AC_CHECK_LIB([ach], [ach_open])

# Checks for header files.
AC_CHECK_HEADERS([string.h sys/ioctl.h sys/socket.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_C_INLINE

# Checks for library functions.
AC_CHECK_FUNCS([clock_gettime memset socket])

AC_CONFIG_FILES([ Makefile ])
AC_OUTPUT
