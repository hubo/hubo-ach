/*

	A simple header file to include in hubo-daemon.c
	and hubo-daemonizer.c

	Author: M.X. Grey ( mxgrey@gatech.edu )

*/




int hubo_sig_quit;
int hubo_sig_usr1;
int hubo_sig_usr2;

int verbose = 0;
int debug = 0;

void hubo_daemonize();
void hubo_daemon_close();
void hubo_assert( int result ); // Instructs the program to quit gracefully if the result is not true
void hubo_assert( int result, const char *msg );
void hubo_verbose( const char *verb );
void hubo_debug( const char *deb );
