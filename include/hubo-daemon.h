/*

	A simple header file to include in hubo-daemon.c
	and hubo-daemonizer.c

	Author: M.X. Grey ( mxgrey@gatech.edu )

*/


#ifdef __cplusplus
extern "C" {
#endif


int hubo_sig_quit;
int hubo_sig_usr1;
int hubo_sig_usr2;

void hubo_daemonize();
void hubo_daemon_close();
void hubo_assert( int result, int line ); // Instructs the program to quit gracefully if the result is not true

#ifdef __cplusplus
}
#endif
