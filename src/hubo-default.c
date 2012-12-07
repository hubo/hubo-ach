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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <errno.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>


#include <string.h>
#include <inttypes.h>

// For ach
#include "ach.h"

// For hubo
#include "hubo.h"

#define NSEC_PER_SEC    1000000000

// ach message type
//typedef struct hubo HUBO[1];

// ach channels
// ach channels
ach_channel_t chan_hubo_ref;      // hubo-ach
ach_channel_t chan_hubo_init_cmd; // hubo-ach-console
ach_channel_t chan_hubo_state;    // hubo-ach-state
ach_channel_t chan_hubo_param;    // hubo-ach-param

void setPosZeros() {
	// open ach channel
//        int r = ach_open(&chan_num, "hubo", NULL);
//        assert( ACH_OK == r );

	struct hubo_ref H;
	memset( &H,   0, sizeof(H));
	size_t fs = 0;
	int r = ach_get( &chan_hubo_ref, &H, sizeof(H), &fs, NULL, ACH_O_LAST );
	//printf("fs = %i, H = %i\n",fs, sizeof(H));
	assert( sizeof(H) == fs );

	int i = 0;
	for( i = 0; i < HUBO_JOINT_COUNT; i++) {
		H.ref[i] = 0.0;
	}
	ach_put(&chan_hubo_ref, &H, sizeof(H));
}

void setConsoleFlags() {
	struct hubo_init_cmd C;
	memset( &C,   0, sizeof(C));
	size_t fs =0;
	int r = ach_get( &chan_hubo_init_cmd, &C, sizeof(C), &fs, NULL, ACH_O_LAST );
	//printf("fs = %i, H = %i\n",fs, sizeof(H));
	assert( sizeof(C) == fs );
	int i = 0;
	for( i = 0; i < HUBO_JOINT_COUNT; i++ ) {
		C.cmd[i] = 0;
		C.val[i] = 0;
	}
	r = ach_put(&chan_hubo_init_cmd, &C, sizeof(C));
}


void setActive() {

	struct hubo_param H;
	memset( &H,   0, sizeof(H));
	size_t fs = 0;
	int r = ach_get( &chan_hubo_param, &H, sizeof(H), &fs, NULL, ACH_O_LAST );
	//printf("fs = %i, H = %i\n",fs, sizeof(H));
	assert( sizeof(H) == fs );

	int i = 0;
	for( i = 0; i < HUBO_JOINT_COUNT; i++ ) {
		H.joint[i].active = false;
		H.joint[i].zeroed = false;
		H.joint[i].refEnc = 0;
	}

	H.sensor[HUBO_FT_R_FOOT].active = true;
	H.sensor[HUBO_FT_L_FOOT].active = true;
	H.sensor[HUBO_FT_R_HAND].active = true;
	H.sensor[HUBO_FT_L_HAND].active = true;
	H.sensor[HUBO_IMU0].active = true;
	H.sensor[HUBO_IMU1].active = true;
	H.sensor[HUBO_IMU2].active = true;

	H.joint[RHY].active = true;
	H.joint[RHR].active = true;
	H.joint[RHP].active = true;
	H.joint[RKN].active = true;
	H.joint[RAP].active = true;
	H.joint[RAR].active = true;

	H.joint[LHY].active = true;
	H.joint[LHR].active = true;
	H.joint[LHP].active = true;
	H.joint[LKN].active = true;
	H.joint[LAP].active = true;
	H.joint[LAR].active = true;

	H.joint[RSP].active = true;
	H.joint[RSR].active = true;
	H.joint[RSY].active = true;
	H.joint[REB].active = true;
	H.joint[RWY].active = true;
	H.joint[RWP].active = false;

	H.joint[LSP].active = true;
	H.joint[LSR].active = true;
	H.joint[LSY].active = true;
	H.joint[LEB].active = true;
	H.joint[LWY].active = true;
	H.joint[LWP].active = false;

	H.joint[NKY].active = false;
	H.joint[NK1].active = false;
	H.joint[NK2].active = false;
	H.joint[WST].active = true;

	H.joint[RF1].active = false;
	H.joint[RF2].active = false;
	H.joint[RF3].active = false;
	H.joint[RF4].active = false;
	H.joint[RF5].active = false;

	H.joint[LF1].active = false;
	H.joint[LF2].active = false;
	H.joint[LF3].active = false;
	H.joint[LF4].active = false;
	H.joint[LF5].active = false;

	ach_put(&chan_hubo_param, &H, sizeof(H));
}

void setName( struct hubo_param *h, int mot, char* name ) {
	sprintf(h->joint[mot].name , "%s" , name );
//	h->joint[mot].name = name;
}

void setDefaults() {
	// open ach channel
//        int r = ach_open(&chan_num, "hubo", NULL);
//        assert( ACH_OK == r );

	struct hubo_param H;
	memset( &H,   0, sizeof(H));
	size_t fs;
	int r = ach_get( &chan_hubo_param, &H, sizeof(H), &fs, NULL, ACH_O_LAST );
	assert( sizeof(H) == fs );

	/* Names */
//	int i = 0;
//	for(i = 0; i < HUBO_JOINT_COUNT; i++) {
//	 	strncpy(H.joint[i].name , "xxx" , 3);
//	}


	sprintf(H.sensor[HUBO_FT_R_FOOT].name , "%s", "FT0");
	sprintf(H.sensor[HUBO_FT_L_FOOT].name , "%s", "FT1" );
	sprintf(H.sensor[HUBO_FT_R_HAND].name , "%s", "FT2" );
	sprintf(H.sensor[HUBO_FT_L_HAND].name , "%s", "FT3" );
	sprintf(H.sensor[HUBO_IMU0].name , "%s", "IMU0" );
	sprintf(H.sensor[HUBO_IMU1].name , "%s", "IMU1" );
	sprintf(H.sensor[HUBO_IMU2].name , "%s", "IMU2" );
	
	printf("%s\n",H.sensor[0].name);
	
	setName(&H , RHY , "RHY"  );
	setName(&H , RHR , "RHR"  );
	setName(&H , RHP , "RHP"  );
	setName(&H , RKN , "RKN"  );
	setName(&H , RAP , "RAP"  );
	setName(&H , RAR , "RAR"  );

	setName(&H , LHY , "LHY"  );
	setName(&H , LHR , "LHR"  );
	setName(&H , LHP , "LHP"  );
	setName(&H , LKN , "LKN"  );
	setName(&H , LAP , "LAP"  );
	setName(&H , LAR , "LAR"  );

	setName(&H , RSP , "RSP"  );
	setName(&H , RSR , "RSR"  );
	setName(&H , RSY , "RSY"  );
	setName(&H , REB , "REB"  );
	setName(&H , RWY , "RWY"  );
	setName(&H , RWP , "RWP"  );

	setName(&H , LSP , "LSP"  );
	setName(&H , LSR , "LSR"  );
	setName(&H , LSY , "LSY"  );
	setName(&H , LEB , "LEB"  );
	setName(&H , LWY , "LWY"  );
	setName(&H , LWP , "LWP"  );

	setName(&H , NKY , "NKY"  );
	setName(&H , NK1 , "NK1"  );
	setName(&H , NK2 , "NK2"  );
	setName(&H , WST , "WST"  );

	setName(&H , RF1 , "RF1"  );
	setName(&H , RF2 , "RF2"  );
	setName(&H , RF3 , "RF3"  );
	setName(&H , RF4 , "RF4"  );
	setName(&H , RF5 , "RF5"  );

	setName(&H , LF1 , "LF1"  );
	setName(&H , LF2 , "LF2"  );
	setName(&H , LF3 , "LF3"  );
	setName(&H , LF4 , "LF4"  );
	setName(&H , LF5 , "LF5"  );


	H.sensor[HUBO_FT_R_FOOT].boardNo = 1;
	H.sensor[HUBO_FT_L_FOOT].boardNo = 2;
	H.sensor[HUBO_FT_R_HAND].boardNo = 6;
	H.sensor[HUBO_FT_L_HAND].boardNo = 7;
	H.sensor[HUBO_IMU0].boardNo = 3;
	H.sensor[HUBO_IMU1].boardNo = 4;
	H.sensor[HUBO_IMU2].boardNo = 5;

	
	H.sensor[HUBO_FT_R_FOOT].sensNo = HUBO_FT_R_FOOT;
	H.sensor[HUBO_FT_L_FOOT].sensNo = HUBO_FT_L_FOOT;
	H.sensor[HUBO_FT_R_HAND].sensNo = HUBO_FT_R_HAND;
	H.sensor[HUBO_FT_L_HAND].sensNo = HUBO_FT_L_HAND;
	H.sensor[HUBO_IMU0].sensNo = HUBO_IMU0;
	H.sensor[HUBO_IMU1].sensNo = HUBO_IMU1;
	H.sensor[HUBO_IMU2].sensNo = HUBO_IMU2;

	H.joint[RHY].jntNo = RHY;
	H.joint[RHR].jntNo = RHR;
	H.joint[RHP].jntNo = RHP;
	H.joint[RKN].jntNo = RKN;
	H.joint[RAP].jntNo = RAP;
	H.joint[RAR].jntNo = RAR;

	H.joint[LHY].jntNo = LHY;
	H.joint[LHR].jntNo = LHR;
	H.joint[LHP].jntNo = LHP;
	H.joint[LKN].jntNo = LKN;
	H.joint[LAP].jntNo = LAP;
	H.joint[LAR].jntNo = LAR;

	H.joint[RSP].jntNo = RSP;
	H.joint[RSR].jntNo = RSR;
	H.joint[RSY].jntNo = RSY;
	H.joint[REB].jntNo = REB;
	H.joint[RWY].jntNo = RWY;
	H.joint[RWP].jntNo = RWP;

	H.joint[LSP].jntNo = LSP;
	H.joint[LSR].jntNo = LSR;
	H.joint[LSY].jntNo = LSY;
	H.joint[LEB].jntNo = LEB;
	H.joint[LWY].jntNo = LWY;
	H.joint[LWP].jntNo = LWP;

	H.joint[NKY].jntNo = NKY;
	H.joint[NK1].jntNo = NK1;
	H.joint[NK2].jntNo = NK2;
	H.joint[WST].jntNo = WST;

	H.joint[RF1].jntNo = RF1;
	H.joint[RF2].jntNo = RF2;
	H.joint[RF3].jntNo = RF3;
	H.joint[RF4].jntNo = RF4;
	H.joint[RF5].jntNo = RF5;

	H.joint[LF1].jntNo = LF1;
	H.joint[LF2].jntNo = LF2;
	H.joint[LF3].jntNo = LF3;
	H.joint[LF4].jntNo = LF4;
	H.joint[LF5].jntNo = LF5;



	/* Drive wheels */
	H.joint[RHY].drive = 10;
	H.joint[RHR].drive = 324;
	H.joint[RHP].drive = 16;
	H.joint[RKN].drive = 16;
	H.joint[RAP].drive = 10;
	H.joint[RAR].drive = 324;

	H.joint[LHY].drive = 10;
	H.joint[LHR].drive = 324;
	H.joint[LHP].drive = 16;
	H.joint[LKN].drive = 16;
	H.joint[LAP].drive = 10;
	H.joint[LAR].drive = 324;

	H.joint[RSP].drive = 11;
	H.joint[RSR].drive = 1;
	H.joint[RSY].drive = 1;
	H.joint[REB].drive = 20;
	H.joint[RWY].drive = 1;
	H.joint[RWP].drive = 1;

	H.joint[LSP].drive = 11;
	H.joint[LSR].drive = 1;
	H.joint[LSY].drive = 1;
	H.joint[LEB].drive = 20;
	H.joint[LWY].drive = 1;
	H.joint[LWP].drive = 1;

	H.joint[NKY].drive = 1;
	H.joint[NK1].drive = 1;
	H.joint[NK2].drive = 1;
	H.joint[WST].drive = 10;

	H.joint[RF1].drive = 1;
	H.joint[RF2].drive = 1;
	H.joint[RF3].drive = 1;
	H.joint[RF4].drive = 1;
	H.joint[RF5].drive = 1;

	H.joint[LF1].drive = 1;
	H.joint[LF2].drive = 1;
	H.joint[LF3].drive = 1;
	H.joint[LF4].drive = 1;
	H.joint[LF5].drive = 1;



	/* Driven Wheels */
	H.joint[RHY].driven = 25;
	H.joint[RHR].driven = 1024;
	H.joint[RHP].driven = 20;
	H.joint[RKN].driven = 16;
	H.joint[RAP].driven = 25;
	H.joint[RAR].driven = 1024;

	H.joint[LHY].driven = 25;
	H.joint[LHR].driven = 1024;
	H.joint[LHP].driven = 20;
	H.joint[LKN].driven = 16;
	H.joint[LAP].driven = 25;
	H.joint[LAR].driven = 1024;

	H.joint[RSP].driven = 16;
	H.joint[RSR].driven = 1;
	H.joint[RSY].driven = 1;
	H.joint[REB].driven = 24;
	H.joint[RWY].driven = 1;
	H.joint[RWP].driven = 1;

	H.joint[LSP].driven = 16;
	H.joint[LSR].driven = 1;
	H.joint[LSY].driven = 1;
	H.joint[LEB].driven = 24;
	H.joint[LWY].driven = 1;
	H.joint[LWP].driven = 1;

	H.joint[NKY].driven = 1;
	H.joint[NK1].driven = 1;
	H.joint[NK2].driven = 1;
	H.joint[WST].driven = 25;

	H.joint[RF1].driven = 1;
	H.joint[RF2].driven = 1;
	H.joint[RF3].driven = 1;
	H.joint[RF4].driven = 1;
	H.joint[RF5].driven = 1;

	H.joint[LF1].driven = 1;
	H.joint[LF2].driven = 1;
	H.joint[LF3].driven = 1;
	H.joint[LF4].driven = 1;
	H.joint[LF5].driven = 1;


	/* Harmonic */
	H.joint[RHY].harmonic = 100;
	H.joint[RHR].harmonic = 160;
	H.joint[RHP].harmonic = 160;
	H.joint[RKN].harmonic = 160;
	H.joint[RAP].harmonic = 100;
	H.joint[RAR].harmonic = 100;

	H.joint[LHY].harmonic = 100;
	H.joint[LHR].harmonic = 160;
	H.joint[LHP].harmonic = 160;
	H.joint[LKN].harmonic = 160;
	H.joint[LAP].harmonic = 100;
	H.joint[LAR].harmonic = 100;

	H.joint[RSP].harmonic = 100;
	H.joint[RSR].harmonic = 100;
	H.joint[RSY].harmonic = 100;
	H.joint[REB].harmonic = 100;
	H.joint[RWY].harmonic = 100;
	H.joint[RWP].harmonic = 100;

	H.joint[LSP].harmonic = 100;
	H.joint[LSR].harmonic = 100;
	H.joint[LSY].harmonic = 100;
	H.joint[LEB].harmonic = 100;
	H.joint[LWY].harmonic = 100;
	H.joint[LWP].harmonic = 100;

	H.joint[NKY].harmonic = 100;
	H.joint[NK1].harmonic = 100;
	H.joint[NK2].harmonic = 100;
	H.joint[WST].harmonic = 100;

	H.joint[RF1].harmonic = 256;
	H.joint[RF2].harmonic = 256;
	H.joint[RF3].harmonic = 256;
	H.joint[RF4].harmonic = 256;
	H.joint[RF5].harmonic = 256;

	H.joint[LF1].harmonic = 256;
	H.joint[LF2].harmonic = 256;
	H.joint[LF3].harmonic = 256;
	H.joint[LF4].harmonic = 256;
	H.joint[LF5].harmonic = 256;


	/* Encoders */
	H.joint[RHY].enc = 4000;
	H.joint[RHR].enc = 4000;
	H.joint[RHP].enc = 4000;
	H.joint[RKN].enc = 4000;
	H.joint[RAP].enc = 4000;
	H.joint[RAR].enc = 4000;

	H.joint[LHY].enc = 4000;
	H.joint[LHR].enc = 4000;
	H.joint[LHP].enc = 4000;
	H.joint[LKN].enc = 4000;
	H.joint[LAP].enc = 4000;
	H.joint[LAR].enc = 4000;

	H.joint[RSP].enc = 4000;
	H.joint[RSR].enc = 4000;
	H.joint[RSY].enc = 4000;
	H.joint[REB].enc = 4000;
	H.joint[RWY].enc = 4000;
	H.joint[RWP].enc = 4000;

	H.joint[LSP].enc = 4000;
	H.joint[LSR].enc = 4000;
	H.joint[LSY].enc = 4000;
	H.joint[LEB].enc = 4000;
	H.joint[LWY].enc = 4000;
	H.joint[LWP].enc = 4000;

	H.joint[NKY].enc = 128;
	H.joint[NK1].enc = 128;
	H.joint[NK2].enc = 128;
	H.joint[WST].enc = 4000;

	H.joint[RF1].enc = 4000;
	H.joint[RF2].enc = 4000;
	H.joint[RF3].enc = 128;
	H.joint[RF4].enc = 128;
	H.joint[RF5].enc = 128;

	H.joint[LF1].enc = 128;
	H.joint[LF2].enc = 128;
	H.joint[LF3].enc = 128;
	H.joint[LF4].enc = 128;
	H.joint[LF5].enc = 128;

	/* CAN channel */


	H.sensor[HUBO_FT_R_FOOT].can = 0;
	H.sensor[HUBO_FT_L_FOOT].can = 0;
	H.sensor[HUBO_FT_R_HAND].can = 1;
	H.sensor[HUBO_FT_L_HAND].can = 1;
	H.sensor[HUBO_IMU0].can = 1;
	H.sensor[HUBO_IMU1].can = 1;
	H.sensor[HUBO_IMU2].can = 1;

	H.joint[RHY].can = 0;
	H.joint[RHR].can = 0;
	H.joint[RHP].can = 0;
	H.joint[RKN].can = 0;
	H.joint[RAP].can = 0;
	H.joint[RAR].can = 0;

	H.joint[LHY].can = 0;
	H.joint[LHR].can = 0;
	H.joint[LHP].can = 0;
	H.joint[LKN].can = 0;
	H.joint[LAP].can = 0;
	H.joint[LAR].can = 0;

	H.joint[RSP].can = 1;
	H.joint[RSR].can = 1;
	H.joint[RSY].can = 1;
	H.joint[REB].can = 1;
	H.joint[RWY].can = 1;
	H.joint[RWP].can = 1;
	H.joint[RWR].can = 1;

	H.joint[LSP].can = 1;
	H.joint[LSR].can = 1;
	H.joint[LSY].can = 1;
	H.joint[LEB].can = 1;
	H.joint[LWY].can = 1;
	H.joint[LWP].can = 1;
	H.joint[LWR].can = 1;

	H.joint[NKY].can = 1;
	H.joint[NK1].can = 1;
	H.joint[NK2].can = 1;
	H.joint[WST].can = 0;

	H.joint[RF1].can = 1;
	H.joint[RF2].can = 1;
	H.joint[RF3].can = 1;
	H.joint[RF4].can = 1;
	H.joint[RF5].can = 1;

	H.joint[LF1].can = 1;
	H.joint[LF2].can = 1;
	H.joint[LF3].can = 1;
	H.joint[LF4].can = 1;
	H.joint[LF5].can = 1;


	/* numMot */
	H.joint[RHY].numMot = 2;
	H.joint[RHR].numMot = 2;
	H.joint[RHP].numMot = 1;
	H.joint[RKN].numMot = 1;
	H.joint[RAP].numMot = 2;
	H.joint[RAR].numMot = 2;

	H.joint[LHY].numMot = 2;
	H.joint[LHR].numMot = 2;
	H.joint[LHP].numMot = 1;
	H.joint[LKN].numMot = 1;
	H.joint[LAP].numMot = 2;
	H.joint[LAR].numMot = 2;

	H.joint[RSP].numMot = 2;
	H.joint[RSR].numMot = 2;
	H.joint[RSY].numMot = 2;
	H.joint[REB].numMot = 2;
	H.joint[RWY].numMot = 2;
	H.joint[RWP].numMot = 2;

	H.joint[LSP].numMot = 2;
	H.joint[LSR].numMot = 2;
	H.joint[LSY].numMot = 2;
	H.joint[LEB].numMot = 2;
	H.joint[LWY].numMot = 2;
	H.joint[LWP].numMot = 2;

	H.joint[NKY].numMot = 3;
	H.joint[NK1].numMot = 3;
	H.joint[NK2].numMot = 3;
	H.joint[WST].numMot = 1;

	H.joint[RF1].numMot = 5;
	H.joint[RF2].numMot = 5;
	H.joint[RF3].numMot = 5;
	H.joint[RF4].numMot = 5;
	H.joint[RF5].numMot = 5;

	H.joint[LF1].numMot = 5;
	H.joint[LF2].numMot = 5;
	H.joint[LF3].numMot = 5;
	H.joint[LF4].numMot = 5;
	H.joint[LF5].numMot = 5;


	/* JMC */
	H.sensor[HUBO_IMU0].canID = IMU0;
	H.sensor[HUBO_IMU1].canID = IMU1;
	H.sensor[HUBO_IMU2].canID = IMU2;
	H.sensor[HUBO_FT_R_FOOT].canID = FT0;
	H.sensor[HUBO_FT_L_FOOT].canID = FT1;
	H.sensor[HUBO_FT_R_HAND].canID = FT2;
	H.sensor[HUBO_FT_L_HAND].canID = FT3;
	H.joint[RHY].jmc = JMC0;
	H.joint[RHR].jmc = JMC0;
	H.joint[RHP].jmc = JMC1;
	H.joint[RKN].jmc = JMC2;
	H.joint[RAP].jmc = JMC3;
	H.joint[RAR].jmc = JMC3;

	H.joint[LHY].jmc = JMC4;
	H.joint[LHR].jmc = JMC4;
	H.joint[LHP].jmc = JMC5;
	H.joint[LKN].jmc = JMC6;
	H.joint[LAP].jmc = JMC7;
	H.joint[LAR].jmc = JMC7;

	H.joint[RSP].jmc = JMC8;
	H.joint[RSR].jmc = JMC8;
	H.joint[RSY].jmc = JMC9;
	H.joint[REB].jmc = JMC9;
	H.joint[RWY].jmc = EJMC0;
	H.joint[RWP].jmc = EJMC0;

	H.joint[LSP].jmc = JMC10;
	H.joint[LSR].jmc = JMC10;
	H.joint[LSY].jmc = JMC11;
	H.joint[LEB].jmc = JMC11;
	H.joint[LWY].jmc = EJMC1;
	H.joint[LWP].jmc = EJMC1;

	H.joint[NKY].jmc = EJMC2;
	H.joint[NK1].jmc = EJMC2;
	H.joint[NK2].jmc = EJMC2;
	H.joint[WST].jmc = EJMC3;

	H.joint[RF1].jmc = EJMC4;
	H.joint[RF2].jmc = EJMC4;
	H.joint[RF3].jmc = EJMC4;
	H.joint[RF4].jmc = EJMC4;
	H.joint[RF5].jmc = EJMC4;

	H.joint[LF1].jmc = EJMC5;
	H.joint[LF2].jmc = EJMC5;
	H.joint[LF3].jmc = EJMC5;
	H.joint[LF4].jmc = EJMC5;
	H.joint[LF5].jmc = EJMC5;

	/* Motor number (out of the total number of motors on the controller */
	H.joint[RHY].motNo = 0;
	H.joint[RHR].motNo = 1;
	H.joint[RHP].motNo = 0;
	H.joint[RKN].motNo = 0;
	H.joint[RAP].motNo = 0;
	H.joint[RAR].motNo = 1;

	H.joint[LHY].motNo = 0;
	H.joint[LHR].motNo = 1;
	H.joint[LHP].motNo = 0;
	H.joint[LKN].motNo = 0;
	H.joint[LAP].motNo = 0;
	H.joint[LAR].motNo = 1;

	H.joint[RSP].motNo = 0;
	H.joint[RSR].motNo = 1;
	H.joint[RSY].motNo = 0;
	H.joint[REB].motNo = 1;
	H.joint[RWY].motNo = 0;
	H.joint[RWP].motNo = 1;

	H.joint[LSP].motNo = 0;
	H.joint[LSR].motNo = 1;
	H.joint[LSY].motNo = 0;
	H.joint[LEB].motNo = 1;
	H.joint[LWY].motNo = 0;
	H.joint[LWP].motNo = 1;

	H.joint[NKY].motNo = 0;
	H.joint[NK1].motNo = 1;
	H.joint[NK2].motNo = 2;
	H.joint[WST].motNo = 0;

	H.joint[RF1].motNo = 0;
	H.joint[RF2].motNo = 1;
	H.joint[RF3].motNo = 2;
	H.joint[RF4].motNo = 3;
	H.joint[RF5].motNo = 4;

	H.joint[LF1].motNo = 0;
	H.joint[LF2].motNo = 1;
	H.joint[LF3].motNo = 2;
	H.joint[LF4].motNo = 3;
	H.joint[LF5].motNo = 4;

	/* Joint Direction */
	H.joint[RHY].dir = 1;
	H.joint[RHR].dir = 1;
	H.joint[RHP].dir = 1;
	H.joint[RKN].dir = 1;
	H.joint[RAP].dir = 1;
	H.joint[RAR].dir = 1;

	H.joint[LHY].dir = 1;
	H.joint[LHR].dir = 1;
	H.joint[LHP].dir = 1;
	H.joint[LKN].dir = 1;
	H.joint[LAP].dir = 1;
	H.joint[LAR].dir = 1;

	H.joint[RSP].dir = 1;
	H.joint[RSR].dir = 1;
	H.joint[RSY].dir = 1;
	H.joint[REB].dir = 1;
	H.joint[RWY].dir = 1;
	H.joint[RWP].dir = 1;

	H.joint[LSP].dir = 1;
	H.joint[LSR].dir = 1;
	H.joint[LSY].dir = 1;
	H.joint[LEB].dir = 1;
	H.joint[LWY].dir = 1;
	H.joint[LWP].dir = 1;

	H.joint[NKY].dir = 1;
	H.joint[NK1].dir = 1;
	H.joint[NK2].dir = 1;
	H.joint[WST].dir = 1;

	H.joint[RF1].dir = 1;
	H.joint[RF2].dir = 1;
	H.joint[RF3].dir = 1;
	H.joint[RF4].dir = 1;
	H.joint[RF5].dir = 1;

	H.joint[LF1].dir = 1;
	H.joint[LF2].dir = 1;
	H.joint[LF3].dir = 1;
	H.joint[LF4].dir = 1;
	H.joint[LF5].dir = 1;

	/* JMC LOCATIONS */
	H.driver[JMC0].jmc[H.joint[RHY].motNo] = RHY;
	H.driver[JMC0].jmc[H.joint[RHR].motNo] = RHR;
	H.driver[JMC1].jmc[H.joint[RHP].motNo] = RHP;
	H.driver[JMC2].jmc[H.joint[RKN].motNo] = RKN;
	H.driver[JMC3].jmc[H.joint[RAP].motNo] = RAP;
	H.driver[JMC3].jmc[H.joint[RAR].motNo] = RAR;

	H.driver[JMC4].jmc[H.joint[LHY].motNo] = LHY;
	H.driver[JMC4].jmc[H.joint[LHR].motNo] = LHR;
	H.driver[JMC5].jmc[H.joint[LHP].motNo] = LHP;
	H.driver[JMC6].jmc[H.joint[LKN].motNo] = LKN;
	H.driver[JMC7].jmc[H.joint[LAP].motNo] = LAP;
	H.driver[JMC7].jmc[H.joint[LAR].motNo] = LAR;

	H.driver[JMC8].jmc[H.joint[RSP].motNo] = RSP;
	H.driver[JMC8].jmc[H.joint[RSR].motNo] = RSR;
	H.driver[JMC9].jmc[H.joint[RSY].motNo] = RSY;
	H.driver[JMC9].jmc[H.joint[REB].motNo] = REB;
	H.driver[EJMC0].jmc[H.joint[RWY].motNo] = RWY;
	H.driver[EJMC0].jmc[H.joint[RWP].motNo] = RWP;

	H.driver[JMC10].jmc[H.joint[LSP].motNo] = LSP;
	H.driver[JMC10].jmc[H.joint[LSR].motNo] = LSR;
	H.driver[JMC11].jmc[H.joint[LSY].motNo] = LSY;
	H.driver[JMC11].jmc[H.joint[LEB].motNo] = LEB;
	H.driver[EJMC1].jmc[H.joint[LWY].motNo] = LWY;
	H.driver[EJMC1].jmc[H.joint[LWP].motNo] = LWP;

	H.driver[EJMC2].jmc[H.joint[NKY].motNo] = NKY;
	H.driver[EJMC2].jmc[H.joint[NK1].motNo] = NK1;
	H.driver[EJMC2].jmc[H.joint[NK2].motNo] = NK2;
	H.driver[EJMC3].jmc[H.joint[WST].motNo] = WST;

	H.driver[EJMC4].jmc[H.joint[RF1].motNo] = RF1;
	H.driver[EJMC4].jmc[H.joint[RF2].motNo] = RF2;
	H.driver[EJMC4].jmc[H.joint[RF3].motNo] = RF3;
	H.driver[EJMC4].jmc[H.joint[RF4].motNo] = RF4;
	H.driver[EJMC4].jmc[H.joint[RF5].motNo] = RF5;

	H.driver[EJMC5].jmc[H.joint[LF1].motNo] = LF1;
	H.driver[EJMC5].jmc[H.joint[LF2].motNo] = LF2;
	H.driver[EJMC5].jmc[H.joint[LF3].motNo] = LF3;
	H.driver[EJMC5].jmc[H.joint[LF4].motNo] = LF4;
	H.driver[EJMC5].jmc[H.joint[LF5].motNo] = LF5;
//	hubo H = {h};
	ach_put(&chan_hubo_param, &H, sizeof(H));

/*
		int i = 0;
		printf("---- first print ----\n");
		for( i = 0; i < HUBO_JOINT_COUNT; i++) {
			printf("i = %i, jnt = %i\n", i, H.joint[i].can);
		}
		printf("---- last print ---\n");
pause();
*/
}




int main(int argc, char **argv){
	(void) argc; (void)argv;

	struct hubo_ref H_ref;
	struct hubo_init_cmd H_init;
	struct hubo_state H_state;
	struct hubo_param H_param;
	memset( &H_ref,   0, sizeof(H_ref));
	memset( &H_init,  0, sizeof(H_init));
	memset( &H_state, 0, sizeof(H_state));
        memset( &H_param, 0, sizeof(H_param));

	size_t fs;
	// open ach channel
	int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME, NULL);
	assert( ACH_OK == r );

	r = ach_open(&chan_hubo_init_cmd, HUBO_CHAN_INIT_CMD_NAME, NULL);
	assert( ACH_OK == r );

	r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME, NULL);
	assert( ACH_OK == r );
	
	r = ach_open(&chan_hubo_param, HUBO_CHAN_PARAM_NAME, NULL);
	assert( ACH_OK == r );
	

	ach_put(&chan_hubo_ref, &H_ref, sizeof(H_ref));
	ach_put(&chan_hubo_init_cmd, &H_init, sizeof(H_init));
	ach_put(&chan_hubo_state, &H_state, sizeof(H_state));
	ach_put(&chan_hubo_param, &H_param, sizeof(H_param));

	setDefaults();
	setPosZeros();
	setActive();
//	r = ach_get( &chan_num, i&H, sizeof(H), &fs, NULL, ACH_O_LAST );
//	assert( sizeof(H) == fs );
/*
	int i = 0;
	for( i = 0; i < HUBO_JOINT_COUNT; i++) {
		printf("jmc-%i = %i\n", i, H.joint[i].jmc);
	}
*/
	printf("hubo - ACH Channel Created\n");
	return 0;

}
