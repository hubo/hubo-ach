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
/* -*-	indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */


#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include "hubo.h"
#include "hubo/hubo-socketcan.h"


hubo_can_t hubo_socket[4];

int hubo_ver_can = 0;

static int openCAN(char* name) {

   	/* Create the socket */
   	int skt = socket( PF_CAN, SOCK_RAW, CAN_RAW );

   	/* Locate the interface you wish to use */
   	struct ifreq ifr;
   	//strcpy(ifr.ifr_name, "vcan0");

	// FIXME: buffer overflow here!!!
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
	hubo_socket[0]  =	skt0;
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
	if( (bytes_sent < 0) & (1 == hubo_ver_can) ) {
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



static int readn (int sockfd, void *buff, size_t n, int timeo){ // microsecond pause
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
				if( 1 == hubo_ver_can ){
					printf("n_read=0\n");
				}
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
	if( (bytes_read < 0) & (1 == hubo_ver_can) ) {
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
