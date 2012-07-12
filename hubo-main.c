#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

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

// ach message type
typedef struct hubo hubo[1];

// ach channels
ach_channel_t chan_num;

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

void getMotorPosFrame(int motor, struct can_frame *frame) {
}

int openCAN(char* name) {

   	/* Create the socket */
   	int skt = socket( PF_CAN, SOCK_RAW, CAN_RAW );
	
   	/* Locate the interface you wish to use */
   	struct ifreq ifr;
   	//strcpy(ifr.ifr_name, "vcan0");
   	strcpy(ifr.ifr_name, name);
   	ioctl(skt, SIOCGIFINDEX, &ifr); /* ifr.ifr_ifindex gets filled
                                  * with that device's index */
   	/* Select that CAN interface, and bind the socket to it. */
   	struct sockaddr_can addr;
   	addr.can_family = AF_CAN;
   	addr.can_ifindex = ifr.ifr_ifindex;
   	bind( skt, (struct sockaddr*)&addr, sizeof(addr) );
	return skt;
}

void huboLoop() {

	
	
	// make can channels
	int skt1 	= 	openCAN("can1");
	int skt0	=	openCAN("can0");
   	
	/* Send a message to the CAN bus */
   	struct can_frame frame;

	// time info
	struct timespec t;
	// int interval = 500000000; // 2hz (0.5 sec)
	int interval = 10000000; // 100 hz (0.01 sec)
	
	// get current time
        //clock_gettime( CLOCK_MONOTONIC,&t);
        clock_gettime( 0,&t);

	sprintf( frame.data, "hello" );
	frame.can_dlc = strlen( frame.data );


	// temp ach channel num;

	int r = 0;
	hubo H;
	while(1) {


		// wait until next shot
		//clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);
		clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);


		// get data from ach
		size_t fs;

		r = ach_get( &chan_num, H, sizeof(H), &fs, NULL, ACH_O_LAST );
		//r = ach_get( &chan_num, H, sizeof(H), &fs, NULL, ACH_O_WAIT|ACH_O_LAST );
		//r = ach_get( &chan_num, h, sizeof(h), &fs, NULL, ACH_O_WAIT );
		assert( sizeof(H) == fs );
		//assert( ACH_OK==r && sizeof(H) == fs );
		//assert( (ACH_OK==r || ACH_MISSED_FRAME==r) && sizeof(H) == fs );
		
		//printf("Id = %f\n",(float)H[0].imu.bno);
       		//frame.can_id = counter++;
       		frame.can_id = 13;
       		int bytes_sent0 = write( skt0, &frame, sizeof(frame) );
       		if( bytes_sent0 < 0 ) {
       	    		perror("bad write");
       		} else {
       	    		//printf("%d bytes sent\n", bytes_sent);
       		}

       		frame.can_id = 14;
       		int bytes_sent1 = write( skt1, &frame, sizeof(frame) );
       		if( bytes_sent1 < 0 ) {
       	    		perror("bad write");
       		} else {
       	    		//printf("%d bytes sent\n", bytes_sent);
       		}

		t.tv_nsec+=interval;
                tsnorm(&t);
	}


}

int main(int argc, char **argv) {

	// RT 
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

	
	// open ach channel
	int r = ach_open(&chan_num, "hubo", NULL);
	assert( ACH_OK == r );

   	
	// run hubo main loop
	int pid_hubo = fork();
	assert(pid_hubo >= 0);
	if(!pid_hubo) huboLoop();

	printf("hubo main loop started\n");

	pause();
	return 0;

}
