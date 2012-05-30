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

// for realtime
#include <sys/mman.h>
//dan
#define MY_PRIORITY (10) /* we use 49 as the PRREMPT_RT use 50
                            as the priority of kernel tasklets
                            and interrupt handler by default */

#define MAX_SAFE_STACK (8*1024) /* The maximum stack size which is
                                   guranteed safe to access without
                                   faulting */



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
 

void stack_prefault(void) {

        unsigned char dummy[MAX_SAFE_STACK];

        memset(dummy, 0, MAX_SAFE_STACK);
        return;
}



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


void runCan(void){
	printf("Start CAN0 Process\n");
	int r = ach_open(&chan_num, "CAN", NULL);
	assert( ACH_OK == r );




	// open serial 
	fd1 = open_port();

	set_serial(fd1);
	// open CAN
	wr=write(fd1,"O\r",2);
	printf("Bytes sent are %d \n",wr);


	struct timespec t;
	int interval = 100000000;  // 10 hz
	

	// for RT
	struct sched_param param;
  	/* Declare ourself as a real time task */

        param.sched_priority = MY_PRIORITY;
        if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
                perror("sched_setscheduler failed");
                exit(-1);
        }

        /* Lock memory */

        if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
                perror("mlockall failed");
                exit(-2);
        }

        /* Pre-fault our stack */

        stack_prefault();

        clock_gettime(CLOCK_MONOTONIC ,&t);


	// get current time
        clock_gettime(0,&t);

        // start one second after
        t.tv_sec++;

	printf("test\n");
	d D = {0};
	while(1){
		// wait until next shot
                //clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&t, NULL);
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


	r = ach_open(&chan_num, "CAN", NULL);
	assert(ACH_OK == r);

	printf("Running CAN0\n");
	// fork processies
//	int pid_runCan0 = fork();
//	assert(pid_runCan0 >=0);
//	if(!pid_runCan0) runCan();
	runCan();
	pause();
	return 0;

}


