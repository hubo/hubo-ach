/* 

	THIS IS A DAEMONIZATION UTILITY SPECIFICALLY FOR HUBO.
	IT HANDLES ALL FORKING, SCHEDULING PRIORITY, AND LOG REDIRECTION.

	Author: M.X. Grey ( mxgrey@gatech.edu )

	Reference: http://www.qgd.unizh.ch/~dpotter/howto/daemonize

*/


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
#include <string.h>
#include <errno.h>
#include "hubo.h"
#include "hubo-daemon.h"

#define DAEMON_NAME "huboDaemon"

// Make sure that the daemon has the permissions it needs
#define RUN_AS_USER "root"
#define LOCKFILE "/var/lock/hubo-daemon" 

// Priority
#define MY_PRIORITY (49)/* we use 49 as the PRREMPT_RT use 50
                            as the priority of kernel tasklets
                            and interrupt handler by default */




#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1

void stack_prefault(void);




static void hubo_sig_handler(int signum)
{
	switch(signum)
	{
		case SIGALRM: exit(EXIT_FAILURE); break;
		case SIGUSR1: hubo_sig_usr1 = 1;  break;
		case SIGUSR2: hubo_sig_usr2 = 1;  break;
		case SIGCHLD: exit(EXIT_FAILURE); break;
		case SIGINT:
		case SIGQUIT:
		case SIGTERM: hubo_sig_quit=1; break;	
	}
}

static void fork_sig_handler(int signum)
{
	switch(signum)
	{
		case SIGUSR1: exit(EXIT_SUCCESS); break;
	}
}

void hubo_daemonize()
{
	syslog( LOG_NOTICE, "Starting daemonization" );


	// Initialize signal variables
	hubo_sig_quit = 0;
	hubo_sig_usr1 = 0;
	hubo_sig_usr2 = 0;


	pid_t pid, child, sid, parent;
	int lfp = -1; // lockfile pointer

	// Already a daemon:
	if( getppid() == 1 ) return; // A value of 1 indicates that there is no parent process


	// Create the lockfile
	if( LOCKFILE && LOCKFILE[0] )
	{
		lfp = open(LOCKFILE,O_RDWR|O_CREAT|O_EXCL,0640);
		if( lfp < 0 )
		{
			syslog( LOG_ERR, "Unable to create lock file %s, code=%d (%s)",
				LOCKFILE, errno, strerror(errno) );
			exit(EXIT_FAILURE);
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
	signal( SIGCHLD, hubo_sig_handler );
//	signal( SIGUSR1, hubo_sig_handler );
	signal( SIGUSR2, hubo_sig_handler );
	signal( SIGALRM, hubo_sig_handler );
	signal( SIGINT,  hubo_sig_handler );
	signal( SIGQUIT, hubo_sig_handler );
	signal( SIGTERM, hubo_sig_handler );


	//Specific to the fork
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
		// or forcibly quit after 2 seconds elapse
		alarm(2);
		pause();
	
		exit( EXIT_FAILURE );
	}
	
	// If child == 0, then we are in the child process
	
	//Second fork
	pid = fork(); // pid will now represent the final process ID of the daemon
	
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
		// or forcibly quit after 2 seconds elapse
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

	signal( SIGUSR1, hubo_sig_handler ); // Return SIGUSR1 to the standard sig handler	
	
	// Change file mode mask
	umask(0);

	// Create a new Session ID for the child process
	sid = setsid();
	if( sid<0 )
	{
		syslog( LOG_ERR, "Unable to create a new session, code=%d (%s)",
			errno, strerror(errno) );
		exit(EXIT_FAILURE);
	}

	
	// Change the current working directory to prevent the currect directory from being locked
	if( (chdir("/")) < 0 )
	{
		syslog( LOG_ERR, "Unable to change directory to %s, code=%d (%s)",
			"/etc/hubo-daemon", errno, strerror(errno) );
		exit(EXIT_FAILURE);
	}


	// Create files for logging, in case they don't exist already
	if(	!fopen( "/etc/hubo-daemon/daemon-output", "w" ) ||
		!fopen( "/etc/hubo-daemon/daemon-error", "w" ) )
	{
		syslog( LOG_ERR, "Unable to create log files, code=%d (%s)",
			errno, strerror(errno) );
		exit(EXIT_FAILURE);
	}

	// Redirect standard files to /dev/hubo-daemon
	if(	!freopen( "/etc/hubo-daemon/daemon-output", "w", stdout ) ||
		!freopen( "/etc/hubo-daemon/daemon-error", "w", stderr ) )
	{
		syslog( LOG_ERR, "Unable to stream output, code=%d (%s)",
			errno, strerror(errno) );
		exit(EXIT_FAILURE);
	}


	// Tell parent process that everything is okay
	kill( parent, SIGUSR1 );

	// RT
	struct sched_param param;
	// Declare ourself as a real time task

	param.sched_priority = MY_PRIORITY;
	if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
		perror("sched_setscheduler failed");
		exit(-1);
	}

	// Lock memory

	if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
		perror("mlockall failed");
		exit(-2);
	}

	// Pre-fault our stack
	stack_prefault();

	syslog( LOG_NOTICE, "Daemonization finished" );

}


void hubo_daemon_close()
{
	//TODO: Shut down everything safely

	remove( LOCKFILE );
	fclose( stdout );
	fclose( stderr );
	syslog( LOG_NOTICE, "Terminated gracefully" );
	closelog();
}


void stack_prefault(void) { 
        unsigned char dummy[MAX_SAFE_STACK]; 
        memset( dummy, 0, MAX_SAFE_STACK ); 
}


void hubo_assert( int result )
{
	if(!result)
	{
		fprintf(stderr, "Assertion failed. code=%d (%s)\n", errno, strerror(errno));
		hubo_daemon_close();
		exit(EXIT_FAILURE);
	}
} 




