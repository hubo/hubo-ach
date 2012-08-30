/* -*-	indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */


#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdio.h>
#include "hubo.h"
#include "hubo-socketcan.h"


hubo_can_t hubo_socket[4];


void openAllCAN(int vCan) {
	int skt1;
	int skt0;
	if(vCan == 1){
		skt1    =       openCAN("vcan1");
		skt0	=	openCAN("vcan0");
	}
	else {
		skt1    =       openCAN("can1");
		skt0	=	openCAN("can0");
	}
	//H.socket[0]   =	skt0;
	hubo_socket[0]          =	skt0;
	//H.socket[1]	=	skt1;
	hubo_socket[1]	=	skt1;

}

/**
 * Sends CAN packet to desired channel
 *
 * @param $first
 *	"@param" is the socket you want to send to
 * @param $second
 *	CAN frame to send
 */
int sendCan(hubo_can_t skt, struct can_frame *f) {
	//int bytes_sent = write( skt, f, sizeof(*f) );
	int bytes_sent = write( skt, f, sizeof(*f) );
	if( bytes_sent < 0 ) {
		perror("bad write");
	} else if( hubo_debug == 1 ) {

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


int readCan(hubo_can_t skt, struct can_frame *f, double timeoD) {
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
	} else if( hubo_debug == 1 ) {
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
