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

	
}


int main(int argc, char **argv){
	(void) argc; (void)argv;
	int r;

/*
	// create chanels
	r = ach_unlink("hubo");
	assert( ACH_OK == r || ACH_ENOENT == r );

	struct hubo h;
	r = ach_create("hubo", 10ul, sizeof(h), NULL);
	//r = ach_create("hubo", 10ul, 256ul, NULL);
	assert( ACH_OK == r);
*/

	struct hubo h;
	setDefaults();

	h.imu.bno = 7;
	h.joint[4].bno = 77;

	hubo H = {h};
	ach_put(&chan_num, H, sizeof(H));


	printf("hubo - ACH Channel Created\n");
	return 0;

}


