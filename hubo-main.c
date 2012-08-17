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
typedef struct hubo h[1];

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


int getEncRef(int jnt, struct hubo h)
{
	return (int)((double)h.joint[jnt].drive/(double)h.joint[jnt].driven/(double)h.joint[jnt].harmonic/(double)h.joint[jnt].enc*2*pi);
}

void fSetFet(int jnt, int onOff, struct hubo *h, struct can_frame *f) {
	
	//struct can_frame f;
	f->can_id 	= CMD_TXDF;	// Set ID
	//char data[3];
	//data[0] 	= (char)h->joint[jnt].jmc;
	//data[1]		= (char)HipEnable;
	//data[2]		= (char)onOff;
	__u8 data[3];
	data[0] 	= h->joint[jnt].jmc;
	data[1]		= HipEnable;
	data[2]		= onOff;
	sprintf(f->data, "%s", data);	
	f->can_dlc = 3; //= strlen( data );	// Set DLC

}

// 28
void fInitializeBoard(int jnt, struct hubo *h, struct can_frame *f) {
	f->can_id 	= CMD_TXDF;
	__u8 data[2];
	data[0] 	= h->joint[jnt].jmc;
	printf("jmc = %i\n",data[0]);
	//data[0] 	= (uint8_t)88;
	data[1] 	= 0xFA;
	sprintf(f->data, "%s", data);
	f->can_dlc = 2;
}

// 10
void fEnableMotorDriver(int jnt, struct hubo *h, struct can_frame *f) {
	f->can_id 	= CMD_TXDF;
	__u8 data[3];
	data[0] 	= (uint8_t)h->joint[jnt].jmc;
	data[1] 	= 0x0B;
	data[2] 	= 0x01;
	sprintf(f->data, "%s", data);
	f->can_dlc = 3;
}

// 13
void fEnableFeedbackController(int jnt, struct hubo *h, struct can_frame *f) {
	f->can_id 	= CMD_TXDF;
	__u8 data[2];
	data[0] 	= (uint8_t)h->joint[jnt].jmc;
	data[1] 	= 0x0E;
	sprintf(f->data, "%s", data);
	f->can_dlc = 2;
}

// 14
void fDisableFeedbackController(int jnt, struct hubo *h, struct can_frame *f) {
	f->can_id 	= CMD_TXDF;
	__u8 data[2];
	data[0] 	= (uint8_t)h->joint[jnt].jmc;
	data[1] 	= 0x0F;
	sprintf(f->data, "%s", data);
	f->can_dlc = 2;
}

// 15
void fSetPositionController(int jnt, struct hubo *h, struct can_frame *f) {
	f->can_id 	= CMD_TXDF;
	__u8 data[3];
	data[0] 	= (uint8_t)h->joint[jnt].jmc;
	data[1] 	= 0x10;
	data[2]		= 0x00;	// position control
	sprintf(f->data, "%s", data);
	f->can_dlc = 3;
}

// 16
void fGotoLimitAndGoOffset(int jnt, struct hubo *h, struct can_frame *f) {
	f->can_id 	= CMD_TXDF;
	__u8 data[8];
	data[0] 	= (uint8_t)h->joint[jnt].jmc;
	data[1] 	= 0x11;
	data[2] 	= ((uint8_t)h->joint[jnt].motNo << 4)|2; // set /DT high
	data[3]  	= 0x00;
	data[4]  	= 0x00;
	data[5]  	= 0x00;
	data[6]  	= 0x00;
	data[7]  	= 0x00;
	sprintf(f->data, "%s", data);
	f->can_dlc = 8;
}


/**
* Sends CAN packet to desired channel
* 
* @param $first
*	"@param" is the socket you want to send to
* @param $second
*	CAN frame to send
*/
int sendCan(int skt, struct can_frame *f) {
	int bytes_sent = write( skt, f, sizeof(*f) );
	if( bytes_sent < 0 ) {
		perror("bad write");
	} else {
		//printf("%d bytes sent\n", bytes_sent);
	}

	return bytes_sent;
}
int readn (int sockfd, void *buff, size_t n, int timeo){ // microsecond pause
	int n_left;
	int n_read;
	char *ptr;
	ptr = buff;
	n_left = n;
	struct timeval timeout;
	fd_set fds;

	timeout.tv_sec = 0;    
  	timeout.tv_usec = timeo; 
  	FD_ZERO(&fds);
  	FD_SET(sockfd, &fds);

	while(n_left>0){
		
		if (select(sockfd+1, &fds, 0, 0, &timeout)>0){ 
			if((n_read=read(sockfd,ptr,n_left))<0){	
				if(errno == EINTR)
					n_read=0;
				else{
					return -1;	
				}
			}
			else if(n_read==0){			
				printf("n_read=0\n");
				break;
			}
			n_left-=n_read;	
			//printf("%s\n", ptr);
			ptr+=n_read;			
			
		}
		else{
			return -1;
		}
	}
	return (n-n_left);
}


int readCan(int skt, struct can_frame *f, double timeoD) {
	// note timeo is the time out in seconds

	int timeo = (int)(timeoD*1000000.0);
	int bytes_read = readn( skt, &f, sizeof(f), timeo );
	//int bytes_read = read( skt, &f, sizeof(f));
	if( bytes_read < 0 ) {
		perror("bad read");
	} else {
		//printf("%d bytes read -- %d:%s\n", bytes_read, frame.can_id, frame.data);
	}
	return bytes_read;
}

void hInitilize(int jnt, struct hubo *h, struct can_frame *f) {
	fInitializeBoard(jnt, h, f);
	//int skt = h->socket[h->joint[jnt].can];
	//sendCan(skt, f);
	sendCan(h->socket[h->joint[jnt].can], f);
//	readCan(h->socket[h->joint[jnt].can], f, 6);
	

}

void hIniAll(struct hubo *H, struct can_frame *f) {
// --std=c99
		printf("2\n");
	int i = 0;	
	for( i = 0; i < numOfJoints; i++ ) {
		if(H->joint[i].active) {
			hInitilize(i, H, f);
			printf("%i\n",i);
		}
	}
}

void huboLoop(int vCan) {
	// get initial values for hubo
	struct hubo H;
	size_t fs;
	int r = ach_get( &chan_num, &H, sizeof(H), &fs, NULL, ACH_O_LAST );
//	printf("fs = %i, H = %i\n",fs, sizeof(H));
 	assert( sizeof(H) == fs );
//	assert( ACH_OK == r );
	
	// make can channels

	int skt1;
	int skt0;
	if(vCan == 1){
		skt1 	= 	openCAN("vcan1");
		skt0	=	openCAN("vcan0");
	}
	else {
		skt1 	= 	openCAN("can1");
		skt0	=	openCAN("can0");
	}
	H.socket[0] 	=	skt0;
	H.socket[1]	=	skt1;
	
	ach_put( &chan_num, &H, sizeof(H));


   	
	/* Send a message to the CAN bus */
   	struct can_frame frame;

	// time info
	struct timespec t;
	int interval = 500000000; // 2hz (0.5 sec)
	//int interval = 10000000; // 100 hz (0.01 sec)
	
	// get current time
        //clock_gettime( CLOCK_MONOTONIC,&t);
        clock_gettime( 0,&t);

	sprintf( frame.data, "1234578" );
	frame.can_dlc = strlen( frame.data );


	int a = 0;
	while(1) {


		// wait until next shot
		//clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);
		clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);



		r = ach_get( &chan_num, &H, sizeof(H), &fs, NULL, ACH_O_LAST );
		assert( sizeof(H) == fs );

//		fSetFet(RSP, 1, H, &frame);
//		frame.can_id = 14;
//		sendCan(skt0, &frame);		

//		hInitilize(RAP, H, &frame);


		if(a == 0) {
			printf("1\n");

			hIniAll(&H, &frame);
			a = 1;
		}



		//hInitilize(REB, &H, &frame);
/*
		int i = 0;
		for( i = 0; i < numOfJoints; i++) {
			printf("i = %i, jnt = %i\n", i, H.joint[i].can);
		}

*/


		t.tv_nsec+=interval;
                tsnorm(&t);
	}


}


int main(int argc, char **argv) {

	int vflag = 0;
	int c;
/* arguements from command line */
	while ((c = getopt (argc, argv, "v")) != -1) {
		switch(c) {
			case 'v':
				vflag = 1;
				break;
			default:
				abort();
		}
	}

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

/*
	int pid_hubo = fork();
	assert(pid_hubo >= 0);
	if(!pid_hubo) huboLoop(vflag);

	printf("hubo main loop started\n");
*/

	huboLoop(vflag);
	pause();
	return 0;

}
