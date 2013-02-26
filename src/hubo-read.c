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
ach_channel_t chan_hubo_init_cmd; // hubo-ach-console
ach_channel_t chan_hubo_state;    // hubo-ach-state

int debug = 0;
int hubo_debug = 1;

void huboLoop() {
	// get initial values for hubo
	hubo_ref_t H_ref;
	hubo_state_t H_state;
	hubo_param_t H_param;
	memset( &H_ref,   0, sizeof(H_ref));
	memset( &H_state, 0, sizeof(H_state));
	memset( &H_param, 0, sizeof(H_param));



	// set default values for Hubo
	setJointParams(&H_param, &H_state);

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

	/* Send a message to the CAN bus */
	struct can_frame frame;

	// time info
	struct timespec t;
	struct timespec t2;
	//int interval = 500000000; // 2hz (0.5 sec)
//        int interval = 100000000; // 10hz (0.1 sec)
	int interval = 50000000; // 20hz (0.2 sec)
	//int interval = 10000000; // 100 hz (0.01 sec)
	//int interval = 5000000; // 200 hz (0.005 sec)
	//int interval = 2000000; // 500 hz (0.002 sec)

	// get current time
	//clock_gettime( CLOCK_MONOTONIC,&t);
	clock_gettime( 0,&t);
	timeb_t tp;
	timeb_t tp_0;
	timeb_t tp_f;
	int a = 0;

	/* get initial tme*/
	ftime(&tp_0);
	double tt = 0.0;
	double f = 0.2;		// frequency
	double T = (double)interval/1000000000.0;
	double A = 1.0;
	double t0 = 0.0;
	double t1 = 0.0;
	int jnt = RHY;
	while(1) {
		// wait until next shot
		clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);

		/* Get latest ACH message */
		r = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
		if(ACH_OK != r) {
			if(debug) {
            	printf("Ref r = %s\n",ach_result_to_string(r));}
			}
		else{   assert( sizeof(H_ref) == fs ); }
		r = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
		if(ACH_OK != r) {
			if(debug) {
            	printf("State r = %s\n",ach_result_to_string(r));}
			}
		else{   assert( sizeof(H_state) == fs ); }

		printf("\033[2J");
		int i = 0;
		int jnt = 0;

		clock_gettime( 0,&t2);
		printf("t = %i.%09i\n",(int)t2.tv_sec,(int)t2.tv_nsec);
		for( i = 0; i < HUBO_JOINT_COUNT; i++) {
			jnt = i;
			if(H_param.joint[jnt].name[0] != 0){
			printf("%-4s: Cmd = %-12f  Ref = %-12f Enc = %-12f Cur = %-12f Tmp = %-12f\n",
			//printf("%-20s: Ref = %-20f \t \t Enc = %-20f \t Cur = %f \t Tmp = %f\n",
				H_param.joint[jnt].name,
				H_ref.ref[jnt],
				H_state.joint[jnt].ref,
				H_state.joint[jnt].pos,
				H_state.joint[jnt].cur,
				H_state.joint[jnt].tmp);
		}}

		i = HUBO_FT_R_FOOT;
		printf("%-4s: Mx = %-12f My = %-12f Fz = %-12f\n",
			H_param.sensor[i].name,
			H_state.ft[i].m_x,
			H_state.ft[i].m_y,
			H_state.ft[i].f_z);

		i = HUBO_FT_L_FOOT;
		printf("%-4s: Mx = %-12f My = %-12f Fz = %-12f\n",
			H_param.sensor[i].name,
			H_state.ft[i].m_x,
			H_state.ft[i].m_y,
			H_state.ft[i].f_z);

		i = HUBO_FT_R_HAND;
		printf("%-4s: Mx = %-12f My = %-12f Fz = %-12f\n",
			H_param.sensor[i].name,
			H_state.ft[i].m_x,
			H_state.ft[i].m_y,
			H_state.ft[i].f_z);

		i = HUBO_FT_L_HAND;
		printf("%-4s: Mx = %-12f My = %-12f Fz = %-12f\n",
			H_param.sensor[i].name,
			H_state.ft[i].m_x,
			H_state.ft[i].m_y,
			H_state.ft[i].f_z);

		i = HUBO_IMU0;
	//TODO: Add in Z gyro? Is it useful?
		printf("%-4s: Ax = %-12f Ay = %-12f Az = %-12f\n",
			H_param.sensor[i].name,
			H_state.imu[0].a_x,
			H_state.imu[0].a_y,
			H_state.imu[0].a_z);

		i = HUBO_IMU1;
		printf("%-4s: Ax = %-12f Ay = %-12f Az = %-12f\n",
			H_param.sensor[i].name,
			H_state.imu[1].a_x,
			H_state.imu[1].a_y,
			H_state.imu[1].a_z);

		i = HUBO_IMU2;
		printf("%-4s: Ax = %-12f Ay = %-12f Wx = %-12f Wy = %-12f\n",
			H_param.sensor[i].name,
			H_state.imu[2].a_x,
			H_state.imu[2].a_y,
			H_state.imu[2].w_x,
			H_state.imu[2].w_y);
	//	printf("REB: Cur = %f \t  Diff = %f \t State = %f \t Ref = %f\n",H_state.joint[jnt].cur, jntDiff, H_state.joint[jnt].pos, H_ref.ref[jnt]);





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

	huboLoop();
	pause();
	return 0;

}
