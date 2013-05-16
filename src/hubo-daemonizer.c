/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu>
 * Date: Feb 03, 2013
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */


#include "hubo-daemonizer.h"


#ifndef RUN_AS_USER
    #define RUN_AS_USER "root"
#endif

int daemon_sig_quit = 0;
int daemon_sig_usr1 = 0;
int daemon_sig_usr2 = 0;

#define LOCKDIR "/var/lock/hubo"
#define LOGDIR "/var/log/hubo"
char lockfile[100] = "/var/lock/hubo/default-daemon";
char gdaemon_name[100] = "default-daemon";


void stack_prefault(void);


static void daemon_sig_handler(int signum)
{
    switch(signum)
    {
        case SIGALRM: exit(EXIT_FAILURE); break;
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


void redirectSigs()
{
    signal( SIGINT,  daemon_sig_handler );
    signal( SIGQUIT, daemon_sig_handler );
    signal( SIGTERM, daemon_sig_handler );
}

void hubo_daemonize(const char *daemon_name, int priority)
{
    char pbuff[100];
    sprintf(gdaemon_name, "%s", daemon_name);
//    sprintf(pbuff, "Starting daemonization for %s", daemon_name);
    syslog( LOG_NOTICE, "Starting daemonization for %s", daemon_name );

    // Initialize signal variables;
    daemon_sig_quit = 0;
    daemon_sig_usr1 = 0;
    daemon_sig_usr2 = 0;


    pid_t pid, child, sid, parent;
    int lfp = -1; // lockfile pointer


    // Already a daemon:
    if( getppid() == 1 ) return; // A value of 1 indicates that there is no parent process

    // Make sure lock directory exists
    struct stat st = {0};
    if( stat(LOCKDIR, &st) == -1 )
        mkdir(LOCKDIR, 0700);
    
    // Create the lockfile
    sprintf(lockfile, "/var/lock/hubo/%s", daemon_name);
    if( lockfile && lockfile[0] )
    {
        lfp = open(lockfile,O_RDWR|O_CREAT|O_EXCL,0640);
        if( lfp < 0 )
        {
            syslog( LOG_ERR, "Unable to create lock file %s, code=%d (%s)"
                             "  Check if daemon already exists!",
                    lockfile, errno, strerror(errno) );
            exit( EXIT_FAILURE );
        }
    }


    // Drop the user if there is one
    if( getuid()==0 || geteuid()==0 )
    {
        struct passwd *pw = getpwnam(RUN_AS_USER);
        if( pw )
        {
            syslog( LOG_NOTICE, "Setting user to " RUN_AS_USER );
            setuid( pw->pw_uid );
        }
    }


    // Redirect the expected signals
    signal( SIGCHLD, daemon_sig_handler );
//	signal( SIGUSR1, daemon_sig_handler );
    signal( SIGUSR2, daemon_sig_handler );
    signal( SIGALRM, daemon_sig_handler );
    redirectSigs();


    // Specific to the fork
    signal( SIGUSR1, fork_sig_handler );

    // First fork
    child = fork();


    // Quit if a child process could not be made
    if( child<0 )
    {
        syslog( LOG_ERR, "Unable to fork daemon, code=%d (%s)",
                errno, strerror(errno) );
        exit( EXIT_FAILURE );
    }

    // Quit if we get a good Process ID
    if( child>0 )
    {
        // Wait for confirmation from the child
        // or forcibly quit after 2 seconds
        alarm(2);
        pause();

        exit( EXIT_FAILURE );
    }

    // If child == 0, then we are in the child process

    // Second fork
    pid = fork(); // pid will now represent the final Process ID of the daemon

    // Quit if the final process could not be made
    if( pid<0 )
    {
        syslog( LOG_ERR, "Unable to fork daemon, code=%d (%s)",
                errno, strerror(errno) );
        exit( EXIT_FAILURE );
    }

    if( pid>0 )
    {
        // Kill the useless parent
        parent = getppid();
        kill( parent, SIGUSR1 );

        // Wait for confirmation from the daemon
        // or forcibly quit after 2 seconds
        alarm(2);
        pause();

        exit( EXIT_FAILURE );
    }

    // If pid == 0, then we are in the final process

    // Now we execute as the final process
    parent = getppid();


    // TODO: Look into correctly handling these signals (SIG_DFL is default and SIG_IGN is ignore):
    signal( SIGCHLD, SIG_DFL ); // Child process dies
    signal( SIGTSTP, SIG_IGN ); // Stop (pause) signal
    signal( SIGTTOU, SIG_IGN ); // Trying to write to terminal
    signal( SIGTTIN, SIG_IGN ); // Trying to read from terminal
    signal( SIGHUP,  SIG_IGN ); // Hangup signal

    signal( SIGUSR1, daemon_sig_handler ); // Return SIGUSR1 to the standard sig handler

    // Change file mode mask
    umask(0);

    // Create a new Session ID for the child process
    sid = setsid();
    if( sid<0 )
    {
        syslog( LOG_ERR, "Unable to create new session, code=%d (%s)",
                errno, strerror(errno) );
        exit( EXIT_FAILURE );
    }

    // Change the current working directory to prevent the current directory from being locked
    if( (chdir("/")) < 0 )
    {
        syslog( LOG_ERR, "Unable to change directory, code=%d (%s)",
                errno, strerror(errno) );
        exit( EXIT_FAILURE );
    }

    struct stat logst = {0};
    if( stat(LOGDIR, &logst) == -1 )
        mkdir(LOGDIR, 0700);

    // Create files for logging, in case they don't exist already
    char outfile[100];
    char errfile[100];
    sprintf(outfile, "/var/log/hubo/%s-output", daemon_name);
    sprintf(errfile,  "/var/log/hubo/%s-error", daemon_name);
    if(     !fopen( outfile, "w" ) ||
            !fopen( errfile, "w" ) )
    {
        syslog( LOG_ERR, "Unable to create log files, code=%d (%s)",
                errno, strerror(errno) );
        exit(EXIT_FAILURE);
    }


    // Redirect standard files to /var/log/daemon/xx
    if(     !freopen( outfile, "w", stdout ) ||
            !freopen( errfile, "w", stderr ) )
    {
        syslog( LOG_ERR, "Unable to stream output, code=%d (%s)",
                errno, strerror(errno) );
        exit( EXIT_FAILURE );
    }


    // Tell parent process that everything is okay
    kill( parent, SIGUSR1 );


    if( priority >= 0 )
    {
        // Real Time
        struct sched_param param;
        // Declare ourself as a real time task

        param.sched_priority = priority;
        if( sched_setscheduler(0, SCHED_FIFO, &param) == -1 )
        {
            fprintf(stderr, "sched_setscheduler Failed!\n");
            exit( EXIT_FAILURE );
        }
    }

    // Lock memory

    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
    {
        fprintf(stderr, "mlockall Failed!\n");
        exit( EXIT_FAILURE );
    }

    // Pre-fault our stack
    stack_prefault();

    syslog( LOG_NOTICE, "Daemonization finished - Process Beginning" );
}


void hubo_daemon_close()
{
    remove( lockfile );
    fclose( stdout );
    fclose( stderr );
    char buff[100];
//    sprintf( buff, "Terminated process %s gracefully", gdaemon_name );
    syslog( LOG_NOTICE, "Terminated process %s gracefully", gdaemon_name );
    closelog();
}


void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];
    memset( dummy, 0, MAX_SAFE_STACK );
}


void hubo_assert( int result, int line )
{
    if(!result)
    {
        fprintf(stderr, "Assertion failed on line %d, at time %s\n\t code=%d (%s)\n",
		line, __TIME__, errno, strerror(errno));
        hubo_daemon_close();
        exit( EXIT_FAILURE );
    }
}




