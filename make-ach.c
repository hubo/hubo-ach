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



int main(int argc, char **argv){
	(void) argc; (void)argv;
	int r;

	// create chanels
	r = ach_unlink("hubo");
	assert( ACH_OK == r || ACH_ENOENT == r );

	r = ach_create("hubo", 10ul, 256ul, NULL);
	assert( ACH_OK == r);


	printf("hubo - ACH Channel Created\n");
	return 0;

}


