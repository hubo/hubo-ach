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

// For Serial
int 	fd1;
int 	fd2;
char	*buff, *buffer, *bufptr;
int	wr, rd, nbytes, tries;
 

// shared memory
typedef	int	d[1];	// data xfer for testing

// ach channels
ach_channel_t chan_num;



static inline void tsnorm(struct timespec *ts){
   	while (ts->tv_nsec >= NSEC_PER_SEC) {
      		ts->tv_nsec -= NSEC_PER_SEC;
      		ts->tv_sec++;
	}
}


void printNum(void){
	int r = ach_open(&chan_num, "getNum", NULL);
	assert( ACH_OK == r );


	struct timespec t;
	int interval = 100000000;


	// open serial 
	fd1 = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY );
	if(fd1 == -1)	{
		printf("Could not open ttyUSB0\n");
	}
	else	{
		print("ttyUSB0 Open\n");
	}

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

		printf("num2 = %f\n", (float)D[0]);



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
	//r = ach_unlink("getNum");
//	r = ach_unlink("getNum");
//	assert( ACH_OK == r || ACH_ENOENT == r );

//	r = ach_create("getNum", 10ul, 256ul, NULL);
//	assert( ACH_OK == r);


	r = ach_open(&chan_num, "getNum", NULL);
	assert(ACH_OK == r);

	printf("Fork on 2");
	// fork processies
	int pid_printNum = fork();
	assert(pid_printNum >=0);
	if(!pid_printNum) printNum();

	pause();
	return 0;

}


