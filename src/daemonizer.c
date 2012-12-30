
#include "daemonizer.h"



void stack_prefault(void);


static void daemon_sig_handler(int signum)
{
    switch(signum)
    {
        case SIGARLM: exit(EXIT_FAILURE); break;
        case SIGUSR1: daemon_sig_usr1 = 1; break;
        case SIGUSR2: daemon_sig_usr2 = 1; break;
        case SIGCHLD: exit(EXIT_FAILURE); break;
        case SIGINT:
        case SIGQUIT:
        case SIGTERM: daemon_sig_quit=1; break;
    }
}

static void fork_sig_handler(int signum)
{
    switch(signum)
    {
        case SIGUSR1: exit(EXIT_SUCCESS); break;
    }
}


void daemonize(const char *daemon_name)
{
    char pbuff[100];

    sprintf(pbuff, "Starting daemonization for %s", daemon_name);


    // Initialize signal variables;
    daemon_sig_quit = 0;
    daemon_sig_usr1 = 0;
    daemon_sig_usr2 = 0;


    pid_t pid, child, sid, parent;
    int lfp = -1; // lockfile pointer


    // Already a daemon:
    if( getppid() == 1 ) return; // A value of 1 indicates that there is no parent process


    // Create the lockfile
    char lockfile[100];
    sprintf(lockfile, "/var/lock/%s", daemon_name);
    if( lockfile && lockfile[0] )
    {
        lfp = open(lockfile,O_RDWR|O_CREAT|O_EXCL,0640);
        if( lfp < 0 )
        {
            syslog( LOG_ERR, "Unable to create lock file %s, code=%d (%s)"
                             "\n\tCheck if daemon already exists!",
                    lockfile, errno, strerror(errno) );
        }
    }
}











































