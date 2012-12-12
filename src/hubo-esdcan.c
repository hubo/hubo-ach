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
#include <stdlib.h>
#include "hubo.h"
#include "hubo/hubo-esdcan.h"


hubo_can_t hubo_socket[4];



const static char *canResultString( int i ) {
    NTCAN_RESULT ntr = i;
    switch( ntr ) {
    case NTCAN_SUCCESS: return "SUCCESS";
    case NTCAN_RX_TIMEOUT: return "RX_TIMEOUT";
    case NTCAN_TX_TIMEOUT: return "TX_TIMEOUT";
    case NTCAN_TX_ERROR: return "TX_ERROR";
    case NTCAN_CONTR_OFF_BUS: return "CONTR_OFF_BUS";
    case NTCAN_CONTR_BUSY: return "CONTR_BUSY";
    case NTCAN_CONTR_WARN: return "CONTR_WARN";
    case NTCAN_NO_ID_ENABLED: return "NO_ID_ENABLED";
    case NTCAN_ID_ALREADY_ENABLED: return "ID_ALREADY_ENABLED";
    case NTCAN_INVALID_FIRMWARE: return "INVALID_FIRMWARE";
    case NTCAN_MESSAGE_LOST: return "MESSAGE_LOST";
    case NTCAN_INVALID_HARDWARE: return "INVALID_HARDWARE";
    case NTCAN_PENDING_WRITE: return "PENDING_WRITE";
    case NTCAN_PENDING_READ: return "PENDING_READ";
    case NTCAN_INVALID_DRIVER: return "INVALID_DRIVER";
    case NTCAN_SOCK_CONN_TIMEOUT: return "SOCK_CONN_TIMEOUT";
    case NTCAN_SOCK_CMD_TIMEOUT: return "SOCK_CMD_TIMEOUT";
    case NTCAN_SOCK_HOST_NOT_FOUND: return "SOCK_HOST_NOT_FOUND";
    case NTCAN_INVALID_PARAMETER: return "INVALID_PARAMETER";
    case NTCAN_INVALID_HANDLE: return "INVALID_HANDLE";
    case NTCAN_NET_NOT_FOUND: return "NET_NOT_FOUND";
    case NTCAN_INSUFFICIENT_RESOURCES: return "INSUFFICIENT_RESOURCES";
    case NTCAN_OPERATION_ABORTED: return "OPERATION_ABORTED";
    case NTCAN_WRONG_DEVICE_STATE: return "WRONG_DEVICE_STATE";
    case NTCAN_HANDLE_FORCED_CLOSE: return "HANDLE_FORCED_CLOSE";
    case NTCAN_NOT_IMPLEMENTED: return "NOT_IMPLEMENTED";
    case NTCAN_NOT_SUPPORTED: return "NOT_SUPPORTED";
    case NTCAN_CONTR_ERR_PASSIVE: return "CONTR_ERR_PASSIVE";
    default: return "unknown";
    }
}


void openAllCAN(int vCan) {
	for ( size_t i = 0; i < 2; i ++ ) {
		int r = canOpen( i, //net
				 0, // flags
				 10, //txqueue
				 10, //rxqueue
				 100, //txtimeout
				 100, //rxtimeout
				 &hubo_socket[i] //handle
			);
		if( NTCAN_SUCCESS != r ) {
			fprintf(stderr, "Unable to open CAN %d: %s\n", i, canResultString(r));
			exit( EXIT_FAILURE );
		}

		r = canSetBaudrate( hubo_socket[i], NTCAN_BAUD_1000 );
		if( NTCAN_SUCCESS != r ) {
			fprintf(stderr, "Unable to set CAN %d baud: %s\n", i, canResultString(r));
			exit( EXIT_FAILURE );
		}
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
	/*** Convert socketcan struct to NTCAN CMSG ***/
	CMSG esd_frame;
	// id
	esd_frame.id = f->can_id;
    if (f->can_id==0x02) printf("Get sensor %d\n",esd_frame.data[0]);
	// len
	esd_frame.len = f->can_dlc;
	// data
	for( size_t i = 0; i < esd_frame.len; i ++ ) {
		esd_frame.data[i] = f->data[i];
	}

	/*** Send the Message ***/
	int32_t num = 1;
	int r = canWrite( skt, &esd_frame, &num, NULL );
	// FIXME: check error and handle failure reasonably
	if (NTCAN_SUCCESS != r) {
		fprintf(stderr, "canWrite error: %s\n", canResultString(r));
	}

}


int readCan(hubo_can_t skt, struct can_frame *f, double timeoD) {
	(void) timeoD; // ignore this

	/*** Get the Message ***/
	CMSG esd_frame;
	int32_t num  = 1;
	int r = canRead( skt, &esd_frame, &num, NULL );
	// FIXME: check error and handle failure reasonably
	if (NTCAN_SUCCESS != r) {
		fprintf(stderr, "canRead error: %s\n", canResultString(r));
		return 0;
	}

	/*** Convert to socketcan struct ***/
	// id
	f->can_id = esd_frame.id;
	// len
	f->can_dlc = esd_frame.len;
	// data
	for( size_t i = 0; i < esd_frame.len; i ++ ) {
		f->data[i] = esd_frame.data[i];
	}

}
