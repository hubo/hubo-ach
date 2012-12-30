#ifndef DAEMONIZER_H
#define DAEMONIZER_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sched.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <syslog.h>
#include <errno.h>
#include <pwd.h>
#include <signal.h>


#define MAX_SAFE_STACK (1024*1024) /* The maximum stack size which is
                   guaranteed safe to access without
                   faulting */

#define RUN_AS_USER "root"
#define MY_PRIORITY (49)

#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1

int daemon_sig_quit;
int daemon_sig_usr1;
int daemon_sig_usr2;


void daemonize(const char *daemon_name);
void daemon_close();
void daemon_assert( int result ); // Instructs the program to quit gracefully if the result is not true
















































#endif // DAEMONIZER_H
