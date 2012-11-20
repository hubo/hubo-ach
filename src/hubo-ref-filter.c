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
#include "../../hubo-ach/include/hubo.h"

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

//#include "../include/hubo_ref_filter.h"
#include "hubo-ref-filter.h"


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


struct timeb {
        time_t   time;
        unsigned short millitm;
        short    timezone;
        short    dstflag;
};



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
ach_channel_t chan_hubo_ref_filter;      // hubo-ach-filter
ach_channel_t chan_hubo_init_cmd; // hubo-ach-console
ach_channel_t chan_hubo_state;    // hubo-ach-state
ach_channel_t chan_hubo_param;    // hubo-ach-param

int debug = 0;
int hubo_debug = 0;
int i = 0;
int j = 0;

void huboLoop() {
        // get initial values for hubo
        struct hubo_ref H_ref;
        struct hubo_ref H_ref_filter;
	struct hubo_ref H_ref_filter_buff[buffLength];
	struct hubo_ref H_ref_filter_buff2[buffLength2];
	struct hubo_state H_state;
	struct hubo_ref H_ref_buff;

	memset( &H_ref,   0, sizeof(H_ref));
	memset( &H_ref_buff,   0, sizeof(H_ref_buff));
	memset( &H_ref_filter,   0, sizeof(H_ref_filter));
	memset( &H_ref_filter_buff,   0, sizeof(H_ref_filter_buff));
	memset( &H_ref_filter_buff2,   0, sizeof(H_ref_filter_buff2));
	memset( &H_state, 0, sizeof(H_state));

	int N = 0;  // counter 1
	int N2 = 0; // counter 2

	printf("buffLength = %i\n",buffLength);
	printf("buffLength2 = %i\n",buffLength2);


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

	r = ach_get( &chan_hubo_ref_filter, &H_ref_filter, sizeof(H_ref_filter), &fs, NULL, ACH_O_LAST );
	if(ACH_OK != r) {
		if(hubo_debug) {
                       	printf("State ini r = %s\n",ach_result_to_string(r));}
		}
	else{   
		assert( sizeof(H_ref_filter) == fs );
	 }
	
	for( i = 0; i< HUBO_JOINT_COUNT; i++) {
		for( j = 0; j < buffLength; j++) {
			H_ref_filter_buff[j].ref[i] = H_ref.ref[i];
			if( j < buffLength2 ) { H_ref_filter_buff2[j].ref[i] = H_ref.ref[i]; }
			H_ref_filter.ref[i] = H_ref.ref[i];
			H_ref_buff.ref[i] = H_ref.ref[i];
		}
	}

        ach_put( &chan_hubo_ref_filter, &H_ref_filter, sizeof(H_ref_filter));

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

	double tmp = 0.0;
	printf("here1\n");
        while(1) {
                // wait until next shot
                clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);

		r = ach_get( &chan_hubo_ref_filter, &H_ref_filter, sizeof(H_ref_filter), &fs, NULL, ACH_O_LAST );
		if( ACH_OK == r ) {
			memcpy(&H_ref_buff, &H_ref_filter, sizeof(H_ref_filter));
		}
		else if(ACH_OK != r) {
			if(hubo_debug) {
               	        	printf("State ini r = %s\n",ach_result_to_string(r));}
			}
		else{   
			assert( sizeof(H_ref_filter) == fs );
	 	}
// ------------------------------------------------------------------------------
// ---------------[ DO NOT EDIT AVBOE THIS LINE]---------------------------------
// ------------------------------------------------------------------------------
		for( i = 0; i < HUBO_JOINT_COUNT; i++) {
			H_ref_filter_buff[N].ref[i] = H_ref_buff.ref[i];
 			// Average
			tmp = 0.0;
			for(j = 0; j < buffLength; j++) { tmp = tmp + H_ref_filter_buff[j].ref[i]; }

			H_ref_filter_buff2[N2].ref[i] = tmp/(double)buffLength;
			H_ref.ref[i] = tmp/(double)buffLength;
		
			tmp = 0.0;	
			for(j = 0; j < buffLength2; j++) { tmp = tmp + H_ref_filter_buff2[j].ref[i]; }


//			H_ref.ref[i] = tmp/(double)buffLength2;
			
			if( N < (buffLength) ) { N = N + 1; }
			else {N = 0;}

			if( N2 < (buffLength2) ) { N2 = N2 + 1; }
			else {N2 = 0;}
		}
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

	r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME , NULL);
        assert( ACH_OK == r );
       
    	/* open ach-filter channel */
    	r = ach_open(&chan_hubo_ref_filter, HUBO_CHAN_REF_FILTER_NAME , NULL);
    	assert( ACH_OK == r );

 
	huboLoop();
        pause();
        return 0;

}
