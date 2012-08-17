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
ach_channel_t chan_num;

void setPosZeros() {
	// open ach channel
//        int r = ach_open(&chan_num, "hubo", NULL);
//        assert( ACH_OK == r );

	struct hubo H;
	size_t fs = 0;
	int r = ach_get( &chan_num, &H, sizeof(H), &fs, NULL, ACH_O_LAST );
	//printf("fs = %i, H = %i\n",fs, sizeof(H));
	assert( sizeof(H) == fs );

	int i = 0;
	for( i = 0; i < numOfJoints; i++) {
		H.joint[i].ref = 0.0;
	}
	ach_put(&chan_num, &H, sizeof(H));
}

void setActive() {

	struct hubo H;
	size_t fs = 0;
	int r = ach_get( &chan_num, &H, sizeof(H), &fs, NULL, ACH_O_LAST );
	//printf("fs = %i, H = %i\n",fs, sizeof(H));
	assert( sizeof(H) == fs );

	int i = 0;
	for( i = 0; i < numOfJoints; i++ ) {
		H.joint[i].active = false;
	}

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
        H.joint[RWP].active = true;

        H.joint[LSP].active = true;
        H.joint[LSR].active = true;
        H.joint[LSY].active = true;
        H.joint[LEB].active = true;
        H.joint[LWY].active = true;
        H.joint[LWP].active = true;

        H.joint[NKY].active = true;
        H.joint[NK1].active = true;
        H.joint[NK2].active = true;
        H.joint[WST].active = true;

        H.joint[RF1].active = true;
        H.joint[RF2].active = true;
        H.joint[RF3].active = true;
        H.joint[RF4].active = true;
        H.joint[RF5].active = true;

        H.joint[LF1].active = true;
        H.joint[LF2].active = true;
        H.joint[LF3].active = true;
        H.joint[LF4].active = true;
        H.joint[LF5].active = true;
	
	ach_put(&chan_num, &H, sizeof(H));
}

void setDefaults() {
	// open ach channel
//        int r = ach_open(&chan_num, "hubo", NULL);
//        assert( ACH_OK == r );

	struct hubo H;
	size_t fs;
	int r = ach_get( &chan_num, &H, sizeof(H), &fs, NULL, ACH_O_LAST );
	assert( sizeof(H) == fs );

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
	H.joint[WST].can = 1;

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
	

	/* JMC */
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

//	hubo H = {h};
	ach_put(&chan_num, &H, sizeof(H));

/*
                int i = 0;
		printf("---- first print ----\n");
                for( i = 0; i < numOfJoints; i++) {
                        printf("i = %i, jnt = %i\n", i, H.joint[i].can);
                }
		printf("---- last print ---\n");
pause();
*/
}


int main(int argc, char **argv){
	(void) argc; (void)argv;

	struct hubo H;
	size_t fs;
	// open ach channel
        int r = ach_open(&chan_num, "hubo", NULL);
        assert( ACH_OK == r );

	ach_put(&chan_num, &H, sizeof(H));
	setDefaults();
	setPosZeros();
	setActive();
	r = ach_get( &chan_num, &H, sizeof(H), &fs, NULL, ACH_O_LAST );
	assert( sizeof(H) == fs );
/*
	int i = 0;
	for( i = 0; i < numOfJoints; i++) {
		printf("jmc-%i = %i\n", i, H.joint[i].jmc);
	}
*/
	printf("hubo - ACH Channel Created\n");
	return 0;

}


