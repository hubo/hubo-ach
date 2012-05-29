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

// for timer
#include <time.h>
#include <sched.h>
#include <sys/io.h>

#include "ach.h"

// for serial
#include <termios.h>
#define NSEC_PER_SEC    1000000000
// For Serial
int 	fd1;
int 	fd2;
char	*buff, *buffer, *bufptr;
int	wr, rd, nbytes, tries;
 

// shared memory
typedef	int	d[1];	// data xfer for testing

// ach channels
ach_channel_t chan_num;
// 
 
void set_serial(int fd){
   struct termios options;

    /*
     * Get the current options for the port...
     */

    tcgetattr(fd, &options);

    /*
     * Set the baud rates to 19200...
     */

    cfsetispeed(&options, B19200);
    cfsetospeed(&options, B19200);

    /*
     * Enable the receiver and set local mode...
     */

    options.c_cflag |= (CLOCAL | CREAD);

    /*
     * Set the new options for the port...
     */

    tcsetattr(fd, TCSANOW, &options);
}

   int open_port(void)
    {
      int fd; /* File descriptor for the port */


      fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
      if (fd == -1)
      {
       /*
	* Could not open the port.
	*/

	perror("open_port: Unable to open /dev/ttyUSB0 - ");
      }
      else
	fcntl(fd, F_SETFL, 0);

      return (fd);
    }




static inline void tsnorm(struct timespec *ts){
   	while (ts->tv_nsec >= NSEC_PER_SEC) {
      		ts->tv_nsec -= NSEC_PER_SEC;
      		ts->tv_sec++;
	}
}


void printNum(void){
	int r = ach_open(&chan_num, "getNum", NULL);
	assert( ACH_OK == r );


	struct timespec t;
	int interval = 1000000000;


	// open serial 
	fd1 = open_port();

	set_serial(fd1);

	// get current time
        clock_gettime(0,&t);

        // start one second after
        t.tv_sec++;


	d D = {0};
	while(1){
		// wait until next shot
                clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);

		//------------------------------
                //------[ do sutff start ]------
		//------------------------------
		
		// get info from ach
		size_t fs;
		r = ach_get( &chan_num, D, sizeof(D), &fs, NULL, ACH_O_WAIT );

		//printf("num2 = %f\n", (float)D[0]);
		wr=write(fd1,"t0126010203040506\r",18);

		printf("Bytes sent are %d \n",wr);
		//------------------------------
		//-----[ do stuff stop ]--------
		//------------------------------
		

		// calculate next shot
                t.tv_nsec+=interval;
                tsnorm(&t);
		
	}
}

int main(int argc, char **argv){
	(void) argc; (void)argv;
	int r;

	// create chanels
	//r = ach_unlink("getNum");
//	r = ach_unlink("getNum");
//	assert( ACH_OK == r || ACH_ENOENT == r );

//	r = ach_create("getNum", 10ul, 256ul, NULL);
//	assert( ACH_OK == r);


	r = ach_open(&chan_num, "getNum", NULL);
	assert(ACH_OK == r);

	printf("Fork on 2");
	// fork processies
	int pid_printNum = fork();
	assert(pid_printNum >=0);
	if(!pid_printNum) printNum();

	pause();
	return 0;

}


