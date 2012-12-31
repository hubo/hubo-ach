
#include "daemonizer.h"


#ifndef RUN_AS_USER
    #define RUN_AS_USER "root"
#endif


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

    sprintf(gdaemon_name, "%s", daemon_name);
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
    signal( SIGINT,  daemon_sig_handler );
    signal( SIGQUIT, daemon_sig_handler );
    signal( SIGTERM, daemon_sig_handler );


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


    // Create files for logging, in case they don't exist already
    char outfile[100];
    char errfile[100];
    sprintf(outfile, "/var/log/daemon/%s-output", daemon_name);
    sprintf(errfile,  "/var/log/daemon/%s-error", daemon_name);
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

    // Real Time
    struct sched_param param;
    // Declare ourself as a real time task

    param.sched_priority = MY_PRIORITY;
    if( sched_setscheduler(0, SCHED_FIFO, &param) == -1 )
    {
        fprintf(stderr, "sched_setscheduler Failed!\n");
        exit( EXIT_FAILURE );
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


void daemon_close()
{
    remove( lockfile );
    fclose( stdout );
    fclose( stderr );
    char buff[100];
    sprintf( buff, "Terminated process %s gracefully", gdaemon_name );
    syslog( LOG_NOTICE, buff );
    closelog();
}


void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];
    memset( dummy, 0, MAX_SAFE_STACK );
}


void daemon_assert( int result )
{
    if(!result)
    {
        fprintf(stderr, "Assertion failed. code=%d (%s)\n", errno, strerror(errno));
        daemon_close();
        exit( EXIT_FAILURE );
    }
}


































