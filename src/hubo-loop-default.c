/* -*-	indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
/*
Copyright (c) 2012, Daniel M. Lofaro
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the author nor the names of its contributors may
      be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
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
#include "hubo.h"

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


typedef struct timeb {
	time_t   time;
	unsigned short millitm;
	short    timezone;
	short    dstflag;
} timeb_t;



/* functions */
void stack_prefault(void);
static inline void tsnorm(struct timespec *ts);
void getMotorPosFrame(int motor, struct can_frame *frame);
void huboLoop();
int ftime(timeb_t *tp);










// ach message type
//typedef struct hubo h[1];

// ach channels
ach_channel_t chan_hubo_ref;      // hubo-ach
ach_channel_t chan_hubo_board_cmd; // hubo-ach-console
ach_channel_t chan_hubo_state;    // hubo-ach-state
ach_channel_t chan_hubo_param;    // hubo-ach-param

int debug = 0;

void huboLoop() {
	// get initial values for hubo
	hubo_ref_t H_ref;
	hubo_state_t H_state;
	hubo_param_t H_param;
	memset( &H_ref,   0, sizeof(H_ref));
	memset( &H_state, 0, sizeof(H_state));
	memset( &H_param, 0, sizeof(H_param));

	size_t fs;
	//int r = ach_get( &chan_hubo_ref, &H, sizeof(H), &fs, NULL, ACH_O_LAST );
	//assert( sizeof(H) == fs );
	int r = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
	if(ACH_OK != r) {
		if(hubo_debug) {
			printf("Ref ini r = %s\n",ach_result_to_string(r));}
		}
	else{   assert( sizeof(H_ref) == fs ); }

	r = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
	if(ACH_OK != r) {
		if(hubo_debug) {
			printf("State ini r = %s\n",ach_result_to_string(r));}
		}
	else{
		assert( sizeof(H_state) == fs );
	 }

	r = ach_get( &chan_hubo_param, &H_param, sizeof(H_param), &fs, NULL, ACH_O_LAST );
	if(ACH_OK != r) {
		if(hubo_debug) {
			printf("State ini r = %s\n",ach_result_to_string(r));}
		}
	else{
		assert( sizeof(H_state) == fs );
	}


	// time info
	struct timespec t;
	//int interval = 500000000; // 2hz (0.5 sec)
	int interval = 10000000; // 100 hz (0.01 sec)
	//int interval = 5000000; // 200 hz (0.005 sec)
	//int interval = 2000000; // 500 hz (0.002 sec)


	/* Sampling Period */
	double T = (double)interval/(double)NSEC_PER_SEC; // (sec)

	// get current time
	//clock_gettime( CLOCK_MONOTONIC,&t);
	clock_gettime( 0,&t);

	while(1) {
		// wait until next shot
		clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);

		/* Get latest ACH message */
		r = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
		if(ACH_OK != r) {
			if(hubo_debug) {
				printf("Ref r = %s\n",ach_result_to_string(r));}
			}
		else{   assert( sizeof(H_ref) == fs ); }
		r = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
		if(ACH_OK != r) {
			if(hubo_debug) {
				printf("State r = %s\n",ach_result_to_string(r));}
			}
		else{   assert( sizeof(H_state) == fs ); }

// ------------------------------------------------------------------------------
// ---------------[ DO NOT EDIT AVBOE THIS LINE]---------------------------------
// ------------------------------------------------------------------------------


			H_ref.ref[RHY] = 1.23456;


// ------------------------------------------------------------------------------
// ---------------[ DO NOT EDIT BELOW THIS LINE]---------------------------------
// ------------------------------------------------------------------------------
		ach_put( &chan_hubo_ref, &H_ref, sizeof(H_ref));
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

	r = ach_open(&chan_hubo_param, HUBO_CHAN_PARAM_NAME , NULL);
	assert( ACH_OK == r );

	r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME , NULL);
	assert( ACH_OK == r );

	huboLoop();
	pause();
	return 0;

}
