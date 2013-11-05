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

#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include "hubo.h"
#include "hubo-daemon.h"
#include "hubo/hubo-socketcan.h"

#include "hubo-io-trace.h"

hubo_can_t hubo_socket[4];

int hubo_ver_can = 0;


int trace_socket = -1; // file descriptor for tracing: -2 means don't deal





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
	if(HUBO_VIRTUAL_MODE_VIRTUAL == vCan | HUBO_VIRTUAL_MODE_OPENHUBO == vCan){
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

	if (trace_socket == -1) { // not opened yet
		trace_socket = iotrace_open(0);
	}

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

    errno = 0;
    int bytes_sent = write( skt, f, sizeof(*f) );

    if (bytes_sent != sizeof(*f)) {
        perror("write");
    }

    if (trace_socket >= 0) {
        io_trace_t trace;
        trace.timestamp = iotrace_gettime();
        trace.is_read = 0;
        trace.fd = skt;
        trace.num_calls = 1;
        trace.transmitted = bytes_sent;
        trace.frame = *f;
        iotrace_write(trace_socket, &trace);
    }

    if (bytes_sent < 0) { bytes_sent = 0; }
    return bytes_sent;

}

int flushCan(hubo_can_t skt, int timeOut, double giveUp)
{
    struct can_frame f;
    int bytes_read;
    double start, time;
    struct timespec ts;
    clock_gettime( CLOCK_MONOTONIC, &ts );
    start = (double)(ts.tv_sec) + (double)(ts.tv_nsec)/1.0E9;

    do
    {
        bytes_read = readCan(skt, &f, timeOut);
        clock_gettime( CLOCK_MONOTONIC, &ts );
        time = (double)(ts.tv_sec) + (double)(ts.tv_nsec)/1.0E9;
    } while(bytes_read > 0 && (time-start)<giveUp);

    return bytes_read;
}

#define NSEC_PER_SEC 1000000000

int readCan(hubo_can_t skt, struct can_frame *f, double timeoD) {

    fd_set read_fds;

    int result;

    // note timeo is the time out in nanoseconds
    int64_t timeo = (int64_t)(timeoD*1e9);
    
    struct timespec timeout;

    int bytes_read = 0;

    FD_ZERO(&read_fds);
    FD_SET(skt, &read_fds);

    timeout.tv_sec = timeo / (int64_t)NSEC_PER_SEC;
    timeout.tv_nsec = timeo % (int64_t)NSEC_PER_SEC;
    
    errno = 0;
    result = pselect(skt+1, &read_fds, 0, 0, &timeout, NULL);

    if (result < 0) {

        if (errno != EINTR) {
            perror("select");
        }

    } else if (result && FD_ISSET(skt, &read_fds)) {
        
        errno = 0;
        bytes_read = read( skt, f, sizeof(*f) );

        if (bytes_read < 0) {
            perror("read");
        }

    }

    if (trace_socket >= 0) {
                
        io_trace_t trace;
        trace.timestamp = iotrace_gettime();
        trace.is_read = 1;
        trace.fd = skt;
        trace.num_calls = 1;
        trace.transmitted = bytes_read;
        trace.frame = *f;

        iotrace_write(trace_socket, &trace);

    }

    // just turn errors into no read
    if (bytes_read < 0) { bytes_read = 0; }

    return bytes_read;

}


/* Local Variables:                          */
/* mode: c                                   */
/* c-basic-offset: 4                         */
/* indent-tabs-mode:  nil                    */
/* End:                                      */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
