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
#include "../hubo-ach/hubo.h"


/* for autocomplete */
#include <sys/types.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/errno.h>

#include <readline/readline.h>
#include <readline/history.h>

#define NSEC_PER_SEC    1000000000

// ach message type
//typedef struct hubo HUBO[1];

// ach channels
ach_channel_t chan_num;

extern char *getwd ();
extern char *xmalloc ();

/* The names of functions that actually do the manipulation. */
int com_list (), com_view (), com_rename (), com_stat (), com_pwd ();
int com_delete (), com_help (), com_cd (), com_quit ();

/* A structure which contains information on the commands this program
   can understand. */

typedef struct {
  char *name;			/* User printable name of the function. */
  Function *func;		/* Function to call to do the job. */
  char *doc;			/* Documentation for this function.  */
} COMMAND;







void consoleLoop(struct hubo *H) {
/* gui for controling basic features of the hubo  */
        printf("hubo-ach - interface 2012-08-18\n");
	ach_put(&chan_num, &H, sizeof(H));

	int fconsole = 1;
	while(fconsole) {

	}
}
// read line library

int main(int argc, char **argv){
	(void) argc; (void)argv;

	struct hubo H;
	size_t fs;
	// open ach channel
        int r = ach_open(&chan_num, "hubo", NULL);
        assert( ACH_OK == r );

	r = ach_get( &chan_num, &H, sizeof(H), &fs, NULL, ACH_O_LAST );
	assert( sizeof(H) == fs );
	
	consoleLoop(&H);
	printf("*** Exiting Hubo-Ach-Console ***\n");
	return 0;

}


