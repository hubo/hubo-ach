/* -*-	indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */

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



/* functions */
void stack_prefault(void);
static inline void tsnorm(struct timespec *ts);
void getMotorPosFrame(int motor, struct can_frame *frame);
int openCAN(char* name);
void setEncRef(int jnt, struct hubo_ref *r, struct hubo_param *h);
void setEncRefAll( struct hubo_ref *r, struct hubo_param *h);
void fSetEncRef(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
void fResetEncoderToZero(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
void fGetCurrentValue(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
void fSetBeep(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f, double beepTime);
void hSetBeep(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f, double beepTime);
void fGetBoardStatusAndErrorFlags(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
void fGetEncoderPos(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
void fInitializeBoard(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
void fEnableMotorDriver(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
void fDisableMotorDriver(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
void fEnableFeedbackController(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
void fDisableFeedbackController(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
void fSetPositionController(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
void fGotoLimitAndGoOffset(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
int sendCan(int skt, struct can_frame *f);
int readn (int sockfd, void *buff, size_t n, int timeo);
int readCan(int skt, struct can_frame *f, double timeoD);
void hInitilizeBoard(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
void hSetEncRef(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
void hSetEncRefAll(struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
void hIniAll(struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
void huboLoop(int vCan);
void hMotorDriverOnOff(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f, int onOff);
void hFeedbackControllerOnOff(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f, int onOff);
void hResetEncoderToZero(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
void huboConsole(struct hubo_ref *r, struct hubo_param *h, struct hubo_init_cmd *c, struct can_frame *f);
void hGotoLimitAndGoOffset(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
uint32_t getEncRef(int jnt, struct hubo_ref *r , struct hubo_param *h);
void hInitializeBoard(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);









// ach message type
//typedef struct hubo h[1];

// ach channels
ach_channel_t chan_hubo_ref;	  // hubo-ach
ach_channel_t chan_hubo_init_cmd; // hubo-ach-console
ach_channel_t chan_hubo_state;    // hubo-ach-state
ach_channel_t chan_hubo_param;    // hubo-ach-param



double hubo_socket[4];
int debug = 0;




void huboLoop(int vCan) {
	int i = 0;  // iterator
	// get initial values for hubo
	struct hubo_ref H_ref;
	struct hubo_init_cmd H_init;
	struct hubo_state H_state;
	struct hubo_param H_param;
	memset( &H_ref,   0, sizeof(H_ref));
	memset( &H_init,  0, sizeof(H_init));
	memset( &H_state, 0, sizeof(H_state));
	memset( &H_param, 0, sizeof(H_param));

	size_t fs;
	int r = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
 	assert( sizeof(H_ref) == fs );
	r = ach_get( &chan_hubo_init_cmd, &H_init, sizeof(H_init), &fs, NULL, ACH_O_LAST );
 	assert( sizeof(H_init) == fs );
	r = ach_get( &chan_hubo_init_cmd, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
 	assert( sizeof(H_state) == fs );
	r = ach_get( &chan_hubo_param, &H_param, sizeof(H_param), &fs, NULL, ACH_O_LAST );
 	assert( sizeof(H_param) == fs );
	
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
	//H.socket[0] 	=	skt0;
	hubo_socket[0] 	=	skt0;
	//H.socket[1]	=	skt1;
	hubo_socket[1]	=	skt1;
	
//	ach_put( &chan_hubo_ref, &H, sizeof(H));


   	
	/* Send a message to the CAN bus */
   	struct can_frame frame;

	// time info
	struct timespec t;
	//int interval = 500000000; // 2hz (0.5 sec)
	//int interval = 10000000; // 100 hz (0.01 sec)
	//int interval = 5000000; // 200 hz (0.005 sec)
	int interval = 2000000; // 500 hz (0.002 sec)
	
	// get current time
        //clock_gettime( CLOCK_MONOTONIC,&t);
        clock_gettime( 0,&t);

	sprintf( frame.data, "1234578" );
	frame.can_dlc = strlen( frame.data );

	int a = 0;
	while(1) {
		// wait until next shot
		clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);
		
		/* Get latest ACH message */
		r = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
		assert( sizeof(H_ref) == fs );

		/* read hubo console */
		huboConsole(&H_ref, &H_param, &H_init, &frame);
		
		// set reference for zeroed joints only	
		for(i = 0; i < HUBO_JOINT_COUNT; i++) {
			if(H_param.joint[i].zeroed == true) {
				hSetEncRef(H_param.joint[i].jntNo, &H_ref, &H_param, &frame);
			}
		}

//		hSetEncRef(RHY, &H, &frame);
//		printf("RHY = %f\n",H.joint[RHY].ref);
		ach_put( &chan_hubo_state, &H_state, sizeof(H_state));
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


uint32_t getEncRef(int jnt, struct hubo_ref *r , struct hubo_param *h) {
	// set encoder from reference
	struct hubo_joint_param *p = &h->joint[jnt];
	return (uint32_t)((double)p->driven/(double)p->drive*(double)p->harmonic*(double)p->enc*(double)r->ref[jnt]/2.0/pi);
}

unsigned long signConvention(long _input) {
	if (_input < 0) return (unsigned long)( ((-_input)&0x007FFFFF) | (1<<23) );
	else return (unsigned long)_input;
}
void fSetEncRef(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f) {
	// set ref
	f->can_id 	= REF_BASE_TXDF + h->joint[jnt].jmc;  //CMD_TXD;F// Set ID
	__u8 data[6];
	uint16_t jmc = h->joint[jnt].jmc;
	if(h->joint[jnt].numMot == 2) {
		__u8 m0 = h->driver[jmc].jmc[0];
		__u8 m1 = h->driver[jmc].jmc[1];
//		printf("m0 = %i, m1= %i \n",m0, m1);
	

		unsigned long pos0 = signConvention((int)getEncRef(m0, r, h));
		unsigned long pos1 = signConvention((int)getEncRef(m1, r, h));
		f->data[0] =    (uint8_t)(pos0		& 0x000000FF);
		f->data[1] = 	(uint8_t)((pos0>>8) 	& 0x000000FF);
		f->data[2] = 	(uint8_t)((pos0>>16)	& 0x000000FF);
		f->data[3] =    (uint8_t)(pos1 		& 0x000000FF);
		f->data[4] = 	(uint8_t)((pos1>>8) 	& 0x000000FF);
		f->data[5] = 	(uint8_t)((pos1>>16)	& 0x000000FF);


/*
		f->data[0] =    (uint8_t)h->joint[m0].refEnc;
		f->data[1] = 	(uint8_t)((h->joint[m0].refEnc & 0x0000ff00) >> 8);
		f->data[2] = 	(uint8_t)((h->joint[m0].refEnc & 0x007f0000) >> 16);
		f->data[3] =    (uint8_t)h->joint[m1].refEnc;
		f->data[4] = 	(uint8_t)((h->joint[m1].refEnc & 0x0000ff00) >> 8);
		f->data[5] = 	(uint8_t)((h->joint[m1].refEnc & 0x007f0000) >> 16);
*/

		//if(h->driver[jmc].jmc[0] < 0) {
		if(r->ref[m0] < 0.0) {
			f->data[2] = f->data[2] | 0x80;
		}
		//if(h->driver[jmc].jmc[1] < 0) {
		if(r->ref[m1] < 0.0) {
			f->data[5] = f->data[5] | 0x80;
		}
	}
	//sprintf(f->data, "%s", data);

	f->can_dlc = 6; //= strlen( data );	// Set DLC
}


// 5
void fResetEncoderToZero(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f) {
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
	f->data[0] 	= h->joint[jnt].jmc;
	f->data[1]		= EncZero;
	f->data[2] 	= h->joint[jnt].motNo;
	//sprintf(f->data, "%s", data);	
	f->can_dlc = 3; //= strlen( data );	// Set DLC
}
// 4
void fGetCurrentValue(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f) {
	// get the value of the current in 10mA units A = V/100
	f->can_id 	= CMD_TXDF;	// Set ID
	__u8 data[2];
	f->data[0] 	= h->joint[jnt].jmc;
	f->data[1]		= SendCurrent;
	f->can_dlc = 2; //= strlen( data );	// Set DLC
}

// 4
void fSetBeep(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f, double beepTime) {
	// get the value of the current in 10mA units A = V/100
	f->can_id 	= CMD_TXDF;	// Set ID
	__u8 data[3];
	f->data[0] 	= h->joint[jnt].jmc;	// BNO
	f->data[1]	= 0x82;			// alarm
	f->data[2]	= (uint8_t)floor(beepTime/0.1);
	f->can_dlc = 3; //= strlen( data );	// Set DLC
}

// 28
void fInitializeBoard(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f) {
	f->can_id 	= CMD_TXDF;
	__u8 data[2];
	f->data[0] 	= h->joint[jnt].jmc;
//	printf("jmc = %i\n",data[0]);
	//data[0] 	= (uint8_t)88;
	f->data[1] 	= 0xFA;
	//sprintf(f->data, "%s", data);
	f->can_dlc = 2;
}

// 10.1
void fEnableMotorDriver(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f) {
	f->can_id 	= CMD_TXDF;
	__u8 data[3];
	f->data[0] 	= (uint8_t)h->joint[jnt].jmc;
	f->data[1] 	= 0x0B;
	f->data[2] 	= 0x01;
	//sprintf(f->data, "%s", data);
	f->can_dlc = 3;
}

// 10.2
void fDisableMotorDriver(int jnt, struct hubo_ref *r, struct hubo_param *h,  struct can_frame *f) {
	f->can_id 	= CMD_TXDF;
	__u8 data[3];
	f->data[0] 	= (uint8_t)h->joint[jnt].jmc;
	f->data[1] 	= 0x0B;
	f->data[2] 	= 0x01;
	//sprintf(f->data, "%s", data);
	f->can_dlc = 3;
}

// 13
void fEnableFeedbackController(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f) {
	f->can_id 	= CMD_TXDF;
	__u8 data[2];
	f->data[0] 	= (uint8_t)h->joint[jnt].jmc;
	f->data[1] 	= 0x0E;
	//sprintf(f->data, "%s", data);
	f->can_dlc = 2;
}

// 14
void fDisableFeedbackController(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f) {
	f->can_id 	= CMD_TXDF;
	__u8 data[2];
	f->data[0] 	= (uint8_t)h->joint[jnt].jmc;
	f->data[1] 	= 0x0F;
	//sprintf(f->data, "%s", data);
	f->can_dlc = 2;
}

// 15
void fSetPositionController(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f) {
	f->can_id 	= CMD_TXDF;
	__u8 data[3];
	f->data[0] 	= (uint8_t)h->joint[jnt].jmc;
	f->data[1] 	= 0x10;
	f->data[2]		= 0x00;	// position control
	//sprintf(f->data, "%s", data);
	f->can_dlc = 3;
}



void hSetBeep(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f, double beepTime) {
	fSetBeep(jnt, r, h, f, beepTime);
	f->can_dlc = 3;
}

// 16  home
void fGotoLimitAndGoOffset(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f) {
	f->can_id 	= CMD_TXDF;
	__u8 data[8];
	f->data[0] 	= (uint8_t)h->joint[jnt].jmc;
	f->data[1] 	= 0x11;
	f->data[2] 	= (((uint8_t)h->joint[jnt].motNo+1) << 4)|2; // set /DT high
	f->data[3]  	= 0x00;
	f->data[4]  	= 0x00;
	f->data[5]  	= 0x00;
	f->data[6]  	= 0x00;
	f->data[7]  	= 0x00;
	//sprintf(f->data, "%s", data);
	f->can_dlc = 8;
//	printf("go home %i\n", ((uint8_t)h->joint[jnt].motNo << 4)|2);
}

void hGotoLimitAndGoOffset(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f) {
	fGotoLimitAndGoOffset(jnt, r, h, f);
	sendCan(hubo_socket[h->joint[jnt].can], f);
	h->joint[jnt].zeroed = true;
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
torDriverOnOff
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

void hInitializeBoard(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f) {
	fInitializeBoard(jnt, r, h, f);
	sendCan(hubo_socket[h->joint[jnt].can], f);
	readCan(hubo_socket[h->joint[jnt].can], f, 4);	// 8 bytes to read and 4 sec timeout
}

void hSetEncRef(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f) {
//	setEncRef(jnt,r, h);
	fSetEncRef(jnt, r, h, f);
	sendCan(hubo_socket[h->joint[jnt].can], f);
//	readCan(h->socket[h->joint[jnt].can], f, 4);	// 8 bytes to read and 4 sec timeout
}

void hIniAll(struct hubo_ref *r, struct hubo_param *h, struct can_frame *f) {
// --std=c99
		printf("2\n");
	int i = 0;	
	for( i = 0; i < HUBO_JOINT_COUNT; i++ ) {
		if(h->joint[i].active) {
			hInitializeBoard(i, r, h, f);
			printf("%i\n",i);
		}
	}
}

void hMotorDriverOnOff(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f, int onOff) {
	if(onOff == 1) { // turn on FET
		fEnableMotorDriver(jnt,r, h, f);
		sendCan(hubo_socket[h->joint[jnt].can], f); }
	else if(onOff == 0) { // turn off FET
		fDisableMotorDriver(jnt,r, h, f);
		sendCan(hubo_socket[h->joint[jnt].can], f); }
}

void hFeedbackControllerOnOff(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f, int onOff) {
	if(onOff == 1) { // turn on FET
		fEnableFeedbackController(jnt,r, h, f);
		sendCan(hubo_socket[h->joint[jnt].can], f); }
	else if(onOff == 0) { // turn off FET
		fDisableFeedbackController(jnt,r, h, f);
		sendCan(hubo_socket[h->joint[jnt].can], f); }
}

void hResetEncoderToZero(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f) {
	fResetEncoderToZero(jnt,r, h, f);
	sendCan(hubo_socket[h->joint[jnt].can], f); 
}
void huboConsole(struct hubo_ref *r, struct hubo_param *h, struct hubo_init_cmd *c, struct can_frame *f) {
	/* gui for controling basic features of the hubo  */
//	printf("hubo-ach - interface 2012-08-18\n");

	size_t fs;
	int status = 0;
	while ( status == 0 | status == ACH_OK | status == ACH_MISSED_FRAME ) {
		/* get oldest ach message */
		//status = ach_get( &chan_hubo_ref, &c, sizeof(c), &fs, NULL, 0 );
		//status = ach_get( &chan_hubo_init_cmd, c, sizeof(*c), &fs, NULL, ACH_O_LAST );
		status = ach_get( &chan_hubo_init_cmd, c, sizeof(*c), &fs, NULL, 0 );
	//printf("here2 h = %f status = %i c = %d v = %f\n", h->joint[0].ref, status,(uint16_t)c->cmd[0], c->val[0]);
//	printf("here2 h = %f status = %i c = %d v = %f\n", h->joint[0].ref, status,(uint16_t)c->cmd[0], c->val[0]);
		if( status == ACH_STALE_FRAMES) {
			break; }
		else {
		//assert( sizeof(c) == fs );
		
//		if ( status != 0 & status != ACH_OK & status != ACH_MISSED_FRAME ) {
//			break; }

			switch (c->cmd[0]) {
				case HUBO_JMC_INI:
					hInitializeBoard(c->cmd[1],r,h,f);
					break;
				case HUBO_FET_ON_OFF:
//					printf("fet on\n");
					hMotorDriverOnOff(c->cmd[1],r,h,f,c->cmd[2]);
					break;
				case HUBO_CTRL_ON_OFF:
					hFeedbackControllerOnOff(c->cmd[1],r,h,f,c->cmd[2]);
					break;
				case HUBO_ZERO_ENC:
					hResetEncoderToZero(c->cmd[1],r,h,f);
					break;
				case HUBO_JMC_BEEP:
					hSetBeep(c->cmd[1],r,h,f,c->val[0]);
					break;
				case HUBO_GOTO_HOME:
					hGotoLimitAndGoOffset(c->cmd[1],r,h,f);
					break;
		//		case HUBO_GOTO_REF:
				default:
					break;
			}
		}
//		printf("c = %i\n",c->cmd[0]);
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

	
	// open hubo reference 
	int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME, NULL);
	assert( ACH_OK == r );

	// open hubo state
	r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME, NULL);
	assert( ACH_OK == r );
   	
	// initilize control channel
	r = ach_open(&chan_hubo_init_cmd, HUBO_CHAN_INIT_CMD_NAME, NULL);
	assert( ACH_OK == r );
	
	// paramater
	r = ach_open(&chan_hubo_param, HUBO_CHAN_PARAM_NAME, NULL);
	assert( ACH_OK == r );
	// run hubo main loop

	
	huboLoop(vflag);
	pause();
	return 0;

}
