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

int debug = 0;

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

/*
uint32_t getEncRef(int jnt, struct hubo *h)
{
	//return (uint32_t)((double)h->joint[jnt].drive/(double)h->joint[jnt].driven/(double)h->joint[jnt].harmonic/(double)h->joint[jnt].enc*2.0*pi);
	return (uint32_t)((double)h->joint[jnt].drive/(double)h->joint[jnt].driven/(double)h->joint[jnt].harmonic/(double)h->joint[jnt].ref*2.0*pi);
}
*/

void setEncRef(int jnt, struct hubo *h) {
	// set encoder from reference
	//h->joint[jnt].enc = (uint32_t)(1675545.2);// (uint32_t)((double)h->joint[jnt].drive/(double)h->joint[jnt].driven/(double)h->joint[jnt].harmonic/(double)h->joint[jnt].ref*2.0*pi);
	h->joint[jnt].refEnc = (uint32_t)((double)h->joint[jnt].driven/(double)h->joint[jnt].drive*(double)h->joint[jnt].harmonic*(double)h->joint[jnt].enc*(double)h->joint[jnt].ref/2.0/pi);
}


// Set Ref
void fSetEncRef(int jnt, struct hubo *h, struct can_frame *f) {
	// set ref
	f->can_id 	= REF_BASE_TXDF + h->joint[jnt].jmc;  //CMD_TXD;F// Set ID
	__u8 data[6];

	uint32_t d 	= 0x00000000;
	d 		= h->joint[jnt].refEnc;
	//uint32_t d2	= d;
	data[0] 	= (uint8_t)( d & 0xff);//	& 0x000000ff); 
	data[1]		= (uint8_t)((d & 0xff00)   >> 8);// 	& 0x0000ff00) >> 8);
	data[2]		= (uint8_t)((0xaaaaaa & 0xff0000) >> 16);//	& 0x00ff0000) >> 16);
	data[3] 	= 0;//4;//(__u8)(d	& 0x000000ff); 
	data[4]		= 0;//5;//(__u8)((d 	& 0x0000ff00) >> 8);
	data[5]		= 0;//6;//(__u8)((d 	& 0x00ff0000) >> 16);
	//sprintf(f->data, "%s", data);

	int i = 0;
	for( i = 0; i < f->can_dlc; i++) {
		f->data[i] = data[i];
	}	
	f->can_dlc = 6; //= strlen( data );	// Set DLC
}


// 9.1
void fEnableFet(int jnt, struct hubo *h, struct can_frame *f) {
	// turn on FETS
	f->can_id 	= CMD_TXDF;	// Set ID
	__u8 data[3];
	data[0] 	= h->joint[jnt].jmc;
	data[1]		= HipEnable;
	data[2]		= 1;
	sprintf(f->data, "%s", data);	
	f->can_dlc = 3; //= strlen( data );	// Set DLC
}

// 5
void fResetEncoderToZero(int jnt, struct hubo *h, struct can_frame *f) {
	/* Reset Encoder to Zero (REZ: 0x06)
	CH: Channel No.
	CH= 0 ~ 4 according to the board.
	CH= 0xF selects ALL Channel
	Action:
	1. Set encoder(s) to Zero.
	2. Initialize internal parameters.
	3. Reset Fault and Error Flags.
	*/

	f->can_id 	= CMD_TXDF;	// Set ID
	__u8 data[3];
	data[0] 	= h->joint[jnt].jmc;
	data[1]		= EncZero;
	data[2] 	= h->joint[jnt].motNo;
	sprintf(f->data, "%s", data);	
	f->can_dlc = 3; //= strlen( data );	// Set DLC
}
// 4
void fGetCurrentValue(int jnt, struct hubo *h, struct can_frame *f) {
	// get the value of the current in 10mA units A = V/100
	f->can_id 	= CMD_TXDF;	// Set ID
	__u8 data[2];
	data[0] 	= h->joint[jnt].jmc;
	data[1]		= SendCurrent;
	sprintf(f->data, "%s", data);	
	f->can_dlc = 2; //= strlen( data );	// Set DLC
}

// 2
void fGetBoardStatusAndErrorFlags(int jnt, struct hubo *h, struct can_frame *f) {
	// get board status and error flags
	f->can_id 	= CMD_TXDF;	// Set ID
	__u8 data[2];
	data[0] 	= h->joint[jnt].jmc;
	data[1]		= SendEncoder;
	sprintf(f->data, "%s", data);	
	f->can_dlc = 2; //= strlen( data );	// Set DLC
}

// 3
void fGetEncoderPos(int jnt, struct hubo *h, struct can_frame *f) {
	// requests Encoder Pos
	f->can_id 	= CMD_TXDF;	// Set ID
	__u8 data[3];
	data[0] 	= h->joint[jnt].jmc;
	data[1]		= SendEncoder;
	data[2]		= 0;
	sprintf(f->data, "%s", data);	
	f->can_dlc = 3; //= strlen( data );	// Set DLC
}

// 9.2
void fDisableFet(int jnt, struct hubo *h, struct can_frame *f) {
	// turn off FETS
	f->can_id 	= CMD_TXDF;	// Set ID
	__u8 data[3];
	data[0] 	= h->joint[jnt].jmc;
	data[1]		= HipEnable;
	data[2]		= 0;
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
	//int bytes_sent = write( skt, f, sizeof(*f) );
	int bytes_sent = write( skt, f, sizeof(*f) );
	if( bytes_sent < 0 ) {
		perror("bad write");
	} else if( debug == 1 ) {

		printf("%d bytes sent -- ", bytes_sent);
		printf(" ID=%i - DLC=%i - Data= ",f->can_id, f->can_dlc);
		int i = 0;
		for(i = 0; i < f->can_dlc; i++) {
			printf(" %i ",f->data[i]);
		}
		printf("\n");
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
	//int bytes_read = readn( skt, &f, sizeof(f), timeo );
//	struct	can_frame F;
/*
	F.data[0] = 3;
	F.data[1] = 3;
	F.data[2] = 3;
	F.data[3] = 3;
	F.data[4] = 3;
	F.data[5] = 3;
	F.data[6] = 3;
*/
// read can with timeout
	int bytes_read = readn( skt, f, sizeof(*f), timeo );
//	int bytes_read = read( skt, &f, sizeof(f));

// this is the working one with no timeout
//	int bytes_read = read( skt, f, sizeof(*f));
	if( bytes_read < 0 ) {
		perror("bad read");
	} else if( debug == 1 ) { 
		printf("%d bytes read -- ", bytes_read);
		int i = 0;
		printf(" ID=%i - DLC=%i - Data= ",f->can_id, f->can_dlc);
		for(i = 0; i < f->can_dlc; i++) {
			printf(" %d ",f->data[i]);
		}
		printf("\n");
	}

	return bytes_read;
}

void hInitilizeBoard(int jnt, struct hubo *h, struct can_frame *f) {
	fInitializeBoard(jnt, h, f);
	sendCan(h->socket[h->joint[jnt].can], f);
	readCan(h->socket[h->joint[jnt].can], f, 4);	// 8 bytes to read and 4 sec timeout
}

void hSetEncRef(int jnt, struct hubo *h, struct can_frame *f) {
	fSetEncRef(jnt, h, f);
	sendCan(h->socket[h->joint[jnt].can], f);
//	readCan(h->socket[h->joint[jnt].can], f, 4);	// 8 bytes to read and 4 sec timeout
}

void hIniAll(struct hubo *H, struct can_frame *f) {
// --std=c99
		printf("2\n");
	int i = 0;	
	for( i = 0; i < numOfJoints; i++ ) {
		if(H->joint[i].active) {
			hInitilizeBoard(i, H, f);
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
		clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);



		r = ach_get( &chan_num, &H, sizeof(H), &fs, NULL, ACH_O_LAST );
		assert( sizeof(H) == fs );

		if(a == 0) {
//			hIniAll(&H, &frame);
			a = 1;
		}
		//hInitilizeBoard(RAP, &H, &frame);
	//	H.joint[RAP].enc = 0xffffcc;
		H.joint[RAP].ref = 0.001;
		setEncRef(RAP,&H);
		printf("ref = %i\n",H.joint[RAP].refEnc);
		hSetEncRef(RAP, &H, &frame);
		t.tv_nsec+=interval;
                tsnorm(&t);
	}


}


int main(int argc, char **argv) {

	int vflag = 0;
	int c;
/* arguements from command line */
/*
	while ((c = getopt (argc, argv, "v")) != -1) {
		switch(c) {
			case 'v':
				vflag = 1;
				break;
			case 'd':
				break;
			default:
				abort();
		}
	}
*/

	int i = 1;
	while(argc > i) {
		if(strcmp(argv[i], "-d") == 0) {
			debug = 1;
		}
		i++;
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
