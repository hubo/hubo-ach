#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <time.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>
#include <stdio.h>

// for timer
#include <time.h>
#include <sched.h>
#include <sys/io.h>
#include <unistd.h>

// for RT
#include <stdlib.h>
#include <sys/mman.h>

// for hubo
#include "../hubo-ach/hubo.h"

// for ach
#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include "ach.h"


/* At time of writing, these constants are not defined in the headers */
#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

/* ... */

/* Somewhere in your app */

// Priority
#define MY_PRIORITY (49)/* we use 49 as the PRREMPT_RT use 50
                            as the priority of kernel tasklets
                            and interrupt handler by default */

#define MAX_SAFE_STACK (1024*1024) /* The maximum stack size which is
                                   guaranteed safe to access without
                                   faulting */


// Timing info
#define NSEC_PER_SEC    1000000000



/* functions */
void stack_prefault(void);
static inline void tsnorm(struct timespec *ts);
void getMotorPosFrame(int motor, struct can_frame *frame);
void huboLoop();
int ftime(struct timeb *tp);










// ach message type
//typedef struct hubo h[1];

// ach channels
ach_channel_t chan_hubo_ref;      // hubo-ach
ach_channel_t chan_hubo_init_cmd; // hubo-ach-console
ach_channel_t chan_hubo_state;    // hubo-ach-state
ach_channel_t chan_hubo_param;    // hubo-ach-param

int debug = 0;

struct timeb {
	time_t   time;
	unsigned short millitm;
       	short    timezone;
	short    dstflag;
};


void huboLoop() {
	// get initial values for hubo
	struct hubo_ref H;
	size_t fs;
	int r = ach_get( &chan_hubo_ref, &H, sizeof(H), &fs, NULL, ACH_O_LAST );
 	assert( sizeof(H) == fs );
   	
	/* Send a message to the CAN bus */
   	struct can_frame frame;

	// time info
	struct timespec t;
	//int interval = 500000000; // 2hz (0.5 sec)
	int interval = 10000000; // 100 hz (0.01 sec)
	//int interval = 5000000; // 200 hz (0.005 sec)
	//int interval = 2000000; // 500 hz (0.002 sec)
	
	// get current time
        //clock_gettime( CLOCK_MONOTONIC,&t);
        clock_gettime( 0,&t);
	struct timeb tp;
	struct timeb tp_0;
	struct timeb tp_f;
	int a = 0;

	/* get initial tme*/
	ftime(&tp_0);
	double tt = 0.0;
	double f = 0.2;		// frequency
	double T = (double)interval/1000000000.0;
	double A = 0.3;
	double t0 = 0.0;
	double t1 = 0.0;
	int jnt = RHY;
	while(1) {
		// wait until next shot
		clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);
		
		/* Get latest ACH message */
		r = ach_get( &chan_hubo_ref, &H, sizeof(H), &fs, NULL, ACH_O_LAST );
		assert( sizeof(H) == fs );

		
		ftime(&tp);
		tp_f.time = tp.time-tp_0.time;
		tp_f.millitm = tp.millitm-tp_0.millitm;
		
		tt = (double)tp_f.time+((int16_t)tp_f.millitm)*0.001;
		

		t1 = t0;
		t0 = tt;
		H.ref[jnt] = A*sin(f*2*pi*tt);


	//	printf("time = %ld.%d %f\n",tp_f.time,tp_f.millitm,tt);
		printf("A = %f\n",H.ref[jnt]);	
		//printf("Diff(t) = %f\n",(t0-t1));
		
		ach_put( &chan_hubo_ref, &H, sizeof(H));
		t.tv_nsec+=interval;
                tsnorm(&t);
	}


}






void stack_prefault(void) {
	unsigned char dummy[MAX_SAFE_STACK];
	memset( dummy, 0, MAX_SAFE_STACK );
}



static inline void tsnorm(struct timespec *ts){

//	clock_nanosleep( NSEC_PER_SEC, TIMER_ABSTIME, ts, NULL);
	// calculates the next shot
        while (ts->tv_nsec >= NSEC_PER_SEC) {
                //usleep(100);	// sleep for 100us (1us = 1/1,000,000 sec)
		ts->tv_nsec -= NSEC_PER_SEC;
                ts->tv_sec++;
        }
}

int main(int argc, char **argv) {

	int vflag = 0;
	int c;

	int i = 1;
	while(argc > i) {
		if(strcmp(argv[i], "-d") == 0) {
			debug = 1;
		}
		i++;
	}

	/* RT */ 
	struct sched_param param;
	/* Declare ourself as a real time task */
        param.sched_priority = MY_PRIORITY;
        if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
                perror("sched_setscheduler failed");
                exit(-1);
        }

        /* Lock memory */
        if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
                perror("mlockall failed");
                exit(-2);
        }

        /* Pre-fault our stack */
        stack_prefault();

	
	/* open ach channel */
	int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME , NULL);
	assert( ACH_OK == r );

	huboLoop();
	pause();
	return 0;

}
