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
typedef struct hubo hubo[1];

// ach channels
ach_channel_t chan_num;

void setPosZeros() {
	// open ach channel
//        int r = ach_open(&chan_num, "hubo", NULL);
//        assert( ACH_OK == r );

	struct hubo h;
	int i = 0;
	for( i = 0; i < numOfJoints; i++) {
		h.joint[i].ref = 0.0;
	}
	hubo H = {h};
	ach_put(&chan_num, H, sizeof(H));
}


void setDefaults() {
	// open ach channel
//        int r = ach_open(&chan_num, "hubo", NULL);
//        assert( ACH_OK == r );

	
	struct hubo h;

	/* Drive wheels */
	h.joint[RHY].drive = 10;
	h.joint[RHR].drive = 324;
	h.joint[RHP].drive = 16;
	h.joint[RKN].drive = 16;
	h.joint[RAP].drive = 10;
	h.joint[RAR].drive = 324;

	h.joint[LHY].drive = 10;
	h.joint[LHR].drive = 324;
	h.joint[LHP].drive = 16;
	h.joint[LKN].drive = 16;
	h.joint[LAP].drive = 10;
	h.joint[LAR].drive = 324;

	h.joint[RSP].drive = 11;
	h.joint[RSR].drive = 1;
	h.joint[RSY].drive = 1;
	h.joint[REB].drive = 20;
	h.joint[RWY].drive = 1;
	h.joint[RWP].drive = 1;
        
	h.joint[LSP].drive = 11;
	h.joint[LSR].drive = 1;
	h.joint[LSY].drive = 1;
	h.joint[LEB].drive = 20;
	h.joint[LWY].drive = 1;
	h.joint[LWP].drive = 1;

	h.joint[NKY].drive = 1;
	h.joint[NK1].drive = 1;
	h.joint[NK2].drive = 1;
	h.joint[WST].drive = 10;

	h.joint[RF1].drive = 1;
	h.joint[RF2].drive = 1;
	h.joint[RF3].drive = 1;
	h.joint[RF4].drive = 1;
	h.joint[RF5].drive = 1;

	h.joint[LF1].drive = 1;
	h.joint[LF2].drive = 1;
	h.joint[LF3].drive = 1;
	h.joint[LF4].drive = 1;
	h.joint[LF5].drive = 1;



	/* Driven Wheels */
	h.joint[RHY].driven = 25;
	h.joint[RHR].driven = 1024;
	h.joint[RHP].driven = 20;
	h.joint[RKN].driven = 16;
	h.joint[RAP].driven = 25;
	h.joint[RAR].driven = 1024;
	
	h.joint[LHY].driven = 25;
	h.joint[LHR].driven = 1024;
	h.joint[LHP].driven = 20;
	h.joint[LKN].driven = 16;
	h.joint[LAP].driven = 25;
	h.joint[LAR].driven = 1024;

	h.joint[RSP].driven = 16;
	h.joint[RSR].driven = 1;
	h.joint[RSY].driven = 1;
	h.joint[REB].driven = 24;
	h.joint[RWY].driven = 1;
	h.joint[RWP].driven = 1;

	h.joint[LSP].driven = 16;
	h.joint[LSR].driven = 1;
	h.joint[LSY].driven = 1;
	h.joint[LEB].driven = 24;
	h.joint[LWY].driven = 1;
	h.joint[LWP].driven = 1;

	h.joint[NKY].driven = 1;
	h.joint[NK1].driven = 1;
	h.joint[NK2].driven = 1;
	h.joint[WST].driven = 25;

	h.joint[RF1].driven = 1;
	h.joint[RF2].driven = 1;
	h.joint[RF3].driven = 1;
	h.joint[RF4].driven = 1;
	h.joint[RF5].driven = 1;

	h.joint[LF1].driven = 1;
	h.joint[LF2].driven = 1;
	h.joint[LF3].driven = 1;
	h.joint[LF4].driven = 1;
	h.joint[LF5].driven = 1;


	/* Harmonic */
	h.joint[RHY].harmonic = 100;
	h.joint[RHR].harmonic = 160;
	h.joint[RHP].harmonic = 160;
	h.joint[RKN].harmonic = 160;
	h.joint[RAP].harmonic = 100;
	h.joint[RAR].harmonic = 100;

	h.joint[LHY].harmonic = 100;
	h.joint[LHR].harmonic = 160;	
	h.joint[LHP].harmonic = 160;
	h.joint[LKN].harmonic = 160;
	h.joint[LAP].harmonic = 100;
	h.joint[LAR].harmonic = 100;

	h.joint[RSP].harmonic = 100;
	h.joint[RSR].harmonic = 100;
	h.joint[RSY].harmonic = 100;
	h.joint[REB].harmonic = 100;
	h.joint[RWY].harmonic = 100;
	h.joint[RWP].harmonic = 100;
        
	h.joint[LSP].harmonic = 100;
	h.joint[LSR].harmonic = 100;
	h.joint[LSY].harmonic = 100;
	h.joint[LEB].harmonic = 100;
	h.joint[LWY].harmonic = 100;
	h.joint[LWP].harmonic = 100;

	h.joint[NKY].harmonic = 100;
	h.joint[NK1].harmonic = 100;
	h.joint[NK2].harmonic = 100;
	h.joint[WST].harmonic = 100;

	h.joint[RF1].harmonic = 256;
	h.joint[RF2].harmonic = 256;
	h.joint[RF3].harmonic = 256;
	h.joint[RF4].harmonic = 256;
	h.joint[RF5].harmonic = 256;

	h.joint[LF1].harmonic = 256;
	h.joint[LF2].harmonic = 256;
	h.joint[LF3].harmonic = 256;
	h.joint[LF4].harmonic = 256;
	h.joint[LF5].harmonic = 256;


	/* Encoders */
	h.joint[RHY].enc = 4000;
	h.joint[RHR].enc = 4000;
	h.joint[RHP].enc = 4000;	
	h.joint[RKN].enc = 4000;
	h.joint[RAP].enc = 4000;
	h.joint[RAR].enc = 4000;
	
	h.joint[LHY].enc = 4000;
	h.joint[LHR].enc = 4000;
	h.joint[LHP].enc = 4000;
	h.joint[LKN].enc = 4000;
	h.joint[LAP].enc = 4000;
	h.joint[LAR].enc = 4000;

	h.joint[RSP].enc = 4000;
	h.joint[RSR].enc = 4000;
	h.joint[RSY].enc = 4000;
	h.joint[REB].enc = 4000;
	h.joint[RWY].enc = 4000;
	h.joint[RWP].enc = 4000;
        
	h.joint[LSP].enc = 4000;
	h.joint[LSR].enc = 4000;
	h.joint[LSY].enc = 4000;
	h.joint[LEB].enc = 4000;
	h.joint[LWY].enc = 4000;
	h.joint[LWP].enc = 4000;

	h.joint[NKY].enc = 128;
	h.joint[NK1].enc = 128;
	h.joint[NK2].enc = 128;
	h.joint[WST].enc = 4000;

	h.joint[RF1].enc = 4000;
	h.joint[RF2].enc = 4000;
	h.joint[RF3].enc = 128;
	h.joint[RF4].enc = 128;
	h.joint[RF5].enc = 128;

	h.joint[LF1].enc = 128;
	h.joint[LF2].enc = 128;
	h.joint[LF3].enc = 128;
	h.joint[LF4].enc = 128;
	h.joint[LF5].enc = 128;

	/* CAN channel */
	h.joint[RHY].can = 0;
	h.joint[RHR].can = 0;
	h.joint[RHP].can = 0;
	h.joint[RKN].can = 0;
	h.joint[RAP].can = 0;
	h.joint[RAR].can = 0;

	h.joint[LHY].can = 0;
	h.joint[LHR].can = 0;
	h.joint[LHP].can = 0;
	h.joint[LKN].can = 0;
	h.joint[LAP].can = 0;
	h.joint[LAR].can = 0;

	h.joint[RSP].can = 1;
	h.joint[RSR].can = 1;
	h.joint[RSY].can = 1;
	h.joint[REB].can = 1;
	h.joint[RWY].can = 1;
	h.joint[RWP].can = 1;	
        
	h.joint[LSP].can = 1;
	h.joint[LSR].can = 1;
	h.joint[LSY].can = 1;
	h.joint[LEB].can = 1;
	h.joint[LWY].can = 1;	
	h.joint[LWP].can = 1;

	h.joint[NKY].can = 1;	
	h.joint[NK1].can = 1;
	h.joint[NK2].can = 1;
	h.joint[WST].can = 1;

	h.joint[RF1].can = 1;
	h.joint[RF2].can = 1;
	h.joint[RF3].can = 1;
	h.joint[RF4].can = 1;
	h.joint[RF5].can = 1;

	h.joint[LF1].can = 1;
	h.joint[LF2].can = 1;
	h.joint[LF3].can = 1;
	h.joint[LF4].can = 1;
	h.joint[LF5].can = 1;
	

	/* JMC */
	h.joint[RHY].jmc = JMC0;
	h.joint[RHR].jmc = JMC0;
	h.joint[RHP].jmc = JMC1;
	h.joint[RKN].jmc = JMC2;
	h.joint[RAP].jmc = JMC3;
	h.joint[RAR].jmc = JMC3;

	h.joint[LHY].jmc = JMC4;
	h.joint[LHR].jmc = JMC4;
	h.joint[LHP].jmc = JMC5;
	h.joint[LKN].jmc = JMC6;
	h.joint[LAP].jmc = JMC7;
	h.joint[LAR].jmc = JMC7;

	h.joint[RSP].jmc = JMC8;
	h.joint[RSR].jmc = JMC8;
	h.joint[RSY].jmc = JMC9;
	h.joint[REB].jmc = JMC9;
	h.joint[RWY].jmc = EJMC0;
	h.joint[RWP].jmc = EJMC0;
        
	h.joint[LSP].jmc = JMC10;
	h.joint[LSR].jmc = JMC10;
	h.joint[LSY].jmc = JMC11;
	h.joint[LEB].jmc = JMC11;
	h.joint[LWY].jmc = EJMC1;
	h.joint[LWP].jmc = EJMC1;

	h.joint[NKY].jmc = EJMC2;
	h.joint[NK1].jmc = EJMC2;
	h.joint[NK2].jmc = EJMC2;
	h.joint[WST].jmc = EJMC3;

	h.joint[RF1].jmc = EJMC4;
	h.joint[RF2].jmc = EJMC4;
	h.joint[RF3].jmc = EJMC4;
	h.joint[RF4].jmc = EJMC4;
	h.joint[RF5].jmc = EJMC4;

	h.joint[LF1].jmc = EJMC5;
	h.joint[LF2].jmc = EJMC5;
	h.joint[LF3].jmc = EJMC5;
	h.joint[LF4].jmc = EJMC5;
	h.joint[LF5].jmc = EJMC5;

	/* Motor number (out of the total number of motors on the controller */
	h.joint[RHY].motNo = 0;
	h.joint[RHR].motNo = 1;
	h.joint[RHP].motNo = 0;
	h.joint[RKN].motNo = 0;
	h.joint[RAP].motNo = 0;
	h.joint[RAR].motNo = 1;

	h.joint[LHY].motNo = 0;
	h.joint[LHR].motNo = 1;
	h.joint[LHP].motNo = 0;
	h.joint[LKN].motNo = 0;
	h.joint[LAP].motNo = 0;
	h.joint[LAR].motNo = 1;

	h.joint[RSP].motNo = 0;
	h.joint[RSR].motNo = 1;
	h.joint[RSY].motNo = 0;
	h.joint[REB].motNo = 1;
	h.joint[RWY].motNo = 0;
	h.joint[RWP].motNo = 1;
        
	h.joint[LSP].motNo = 0;
	h.joint[LSR].motNo = 1;
	h.joint[LSY].motNo = 0;
	h.joint[LEB].motNo = 1;
	h.joint[LWY].motNo = 0;
	h.joint[LWP].motNo = 1;

	h.joint[NKY].motNo = 0;
	h.joint[NK1].motNo = 1;
	h.joint[NK2].motNo = 2;
	h.joint[WST].motNo = 0;

	h.joint[RF1].motNo = 0;
	h.joint[RF2].motNo = 1;
	h.joint[RF3].motNo = 2;
	h.joint[RF4].motNo = 3;
	h.joint[RF5].motNo = 4;

	h.joint[LF1].motNo = 0;
	h.joint[LF2].motNo = 1;
	h.joint[LF3].motNo = 2;
	h.joint[LF4].motNo = 3;
	h.joint[LF5].motNo = 4;

	/* Joint Direction */
	h.joint[RHY].dir = 1;
	h.joint[RHR].dir = 1;
	h.joint[RHP].dir = 1;
	h.joint[RKN].dir = 1;
	h.joint[RAP].dir = 1;
	h.joint[RAR].dir = 1;

	h.joint[LHY].dir = 1;
	h.joint[LHR].dir = 1;
	h.joint[LHP].dir = 1;
	h.joint[LKN].dir = 1;
	h.joint[LAP].dir = 1;
	h.joint[LAR].dir = 1;

	h.joint[RSP].dir = 1;
	h.joint[RSR].dir = 1;
	h.joint[RSY].dir = 1;
	h.joint[REB].dir = 1;
	h.joint[RWY].dir = 1;
	h.joint[RWP].dir = 1;
        
	h.joint[LSP].dir = 1;
	h.joint[LSR].dir = 1;
	h.joint[LSY].dir = 1;
	h.joint[LEB].dir = 1;
	h.joint[LWY].dir = 1;
	h.joint[LWP].dir = 1;

	h.joint[NKY].dir = 1;
	h.joint[NK1].dir = 1;
	h.joint[NK2].dir = 1;
	h.joint[WST].dir = 1;

	h.joint[RF1].dir = 1;
	h.joint[RF2].dir = 1;
	h.joint[RF3].dir = 1;
	h.joint[RF4].dir = 1;
	h.joint[RF5].dir = 1;

	h.joint[LF1].dir = 1;
	h.joint[LF2].dir = 1;
	h.joint[LF3].dir = 1;
	h.joint[LF4].dir = 1;
	h.joint[LF5].dir = 1;

	hubo H = {h};
	ach_put(&chan_num, H, sizeof(H));

}


int main(int argc, char **argv){
	(void) argc; (void)argv;

	// open ach channel
        int r = ach_open(&chan_num, "hubo", NULL);
        assert( ACH_OK == r );

//	setDefaults();
	setPosZeros();

	printf("hubo - ACH Channel Created\n");
	return 0;

}


