#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <errno.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>

#include <string.h>
#include <inttypes.h>

// for timer
#include <time.h>
#include <sched.h>
#include <sys/io.h>

#include "ach.h"

#define NSEC_PER_SEC    1000000000

typedef	int	d[1];	// data xfer for testing

// ach channels
ach_channel_t chan_num;



static inline void tsnorm(struct timespec *ts){
   	while (ts->tv_nsec >= NSEC_PER_SEC) {
      		ts->tv_nsec -= NSEC_PER_SEC;
      		ts->tv_sec++;
	}
}


double now() {
	struct timespec t;
	clock_gettime( CLOCK_MONOTONIC, &t );
	return (double)(t.tv_sec) + (double)t.tv_nsec / 1e9;
}

double setNum(void){
	int r =  ach_open(&chan_num, "can", NULL);
	assert( ACH_OK == r);
	int i = 0;
	while(1){
		i++;
		d D = {i};
		ach_put(&chan_num, D, sizeof(D));
		usleep((int)1e6 * 1e-1);
	}
}

void printNum(void){
	int r = ach_open(&chan_num, "can", NULL);
	assert( ACH_OK == r );


	struct timespec t;
	int interval = 500000000;



      	// get current time
        clock_gettime(0,&t);

        // start one second after
        t.tv_sec++;


	d D = {0};
	while(1){
		// wait until next shot
                clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);

		//------------------------------
                //------[ do sutff start ]------
		//------------------------------
		
		// get info from ach
		size_t fs;
		r = ach_get( &chan_num, D, sizeof(D), &fs, NULL, ACH_O_WAIT );

		printf("num = %f\n", (float)D[0]);



		//------------------------------
		//-----[ do stuff stop ]--------
		//------------------------------
		

		// calculate next shot
                t.tv_nsec+=interval;
                tsnorm(&t);
		
	}
}

int main(int argc, char **argv){
	(void) argc; (void)argv;
	int r;

	// create chanels
	r = ach_unlink("can");
	assert( ACH_OK == r || ACH_ENOENT == r );

	r = ach_create("can", 10ul, 256ul, NULL);
	assert( ACH_OK == r);


	// fork processies
	int pid_setNum = fork();
	assert(pid_setNum >=0);
	if(!pid_setNum) setNum();

	int pid_printNum = fork();
	assert(pid_printNum >=0);
	if(!pid_printNum) printNum();

	pause();
	return 0;

}


