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


void setDefaults() {
	// open ach channel
        int r = ach_open(&chan_num, "hubo", NULL);
        assert( ACH_OK == r );

	
	struct hubo h;

	/* Driven Wheels */
	h.joint[RHY].driven = 25.0;
	h.joint[RHR].driven = 1024.0;
	h.joint[RHP].driven = 20.0;
	h.joint[RKN].driven = 16.0;
	h.joint[RAP].driven = 25.0;
	h.joint[RAR].driven = 1024;
	
	h.joint[LHY].driven = 25.0;
	h.joint[LHR].driven = 1024.0;
	h.joint[LHP].driven = 20.0;
	h.joint[LKN].driven = 16.0;
	h.joint[LAP].driven = 25.0;
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


	h.imu.bno = 7;
	h.joint[4].bno = 77;

	hubo H = {h};
	ach_put(&chan_num, H, sizeof(H));

}


int main(int argc, char **argv){
	(void) argc; (void)argv;

	// open ach channel
        int r = ach_open(&chan_num, "hubo", NULL);
        assert( ACH_OK == r );

	setDefaults();


	printf("hubo - ACH Channel Created\n");
	return 0;

}


