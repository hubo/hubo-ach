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

/* At time of writing, these constants are not defined in the headers */
#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

/* ... */

/* Somewhere in your app */




// Timing info
#define NSEC_PER_SEC    1000000000


static inline void tsnorm(struct timespec *ts){
        while (ts->tv_nsec >= NSEC_PER_SEC) {
                ts->tv_nsec -= NSEC_PER_SEC;
                ts->tv_sec++;
        }
}

//void getMotorPosFrame(



int main(int argc, char **argv) {
   	/* Create the socket */
   	int skt = socket( PF_CAN, SOCK_RAW, CAN_RAW );
   	//int skt = socket( PF_CAN, SOCK_RAW, 0 );

   	/* Locate the interface you wish to use */
   	struct ifreq ifr;
   	//strcpy(ifr.ifr_name, "vcan0");
   	strcpy(ifr.ifr_name, "can0");
   	ioctl(skt, SIOCGIFINDEX, &ifr); /* ifr.ifr_ifindex gets filled
                                  * with that device's index */

   	/* Select that CAN interface, and bind the socket to it. */
   	struct sockaddr_can addr;
   	addr.can_family = AF_CAN;
   	addr.can_ifindex = ifr.ifr_ifindex;
   	bind( skt, (struct sockaddr*)&addr, sizeof(addr) );

//	printf("CAN_CALC_BITTIMING = %i",CAN_CALC_BITTIMING);

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

	while(1) {

		// wait until next shot
		//clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);
		clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);

       		//frame.can_id = counter++;
       		frame.can_id = 13;
       		int bytes_sent = write( skt, &frame, sizeof(frame) );
       		if( bytes_sent < 0 ) {
       	    		perror("bad write");
       		} else {
       	    		//printf("%d bytes sent\n", bytes_sent);
       		}

		t.tv_nsec+=interval;
                tsnorm(&t);
       //	sleep(1);

/*
	printf("Press enter to continue\n");
	char enter = 0;
	while (enter != '\r' && enter != '\n') { enter = getchar(); }
	printf("Thank you for pressing enter\n");
*/

	}
}
