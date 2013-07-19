/* -*-	indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
/*
Copyright (c) 2013, Daniel M. Lofaro <dan (at) danLofaro.com>
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

/* Standard Stuff */
#include <string.h>
#include <stdio.h>
#include <time.h>

/* Required Hubo Headers */
#include <hubo.h>

/* For Ach IPC */
#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include "ach.h"

#include <sys/statvfs.h>

#define HUBO_ACH_LOG_GB_BREAK 1.0 // space left in GB to stop the logger
#define HUBO_ACH_LOG_CHECK_TIME 60 // check the space ever X seconds


/* Ach Channel IDs */
ach_channel_t chan_hubo_ref;      // Feed-Forward (Reference)
ach_channel_t chan_hubo_state;    // Feed-Back (State)
ach_channel_t chan_hubo_to_sim;   // To Sim (Trigger)


double getSpaceLeft(char* filename) {
// Returns free space of drive where *filename is on in GB
  struct statvfs buf;
  if (!statvfs(filename, &buf)) {
  unsigned long blksize, blocks, freeblks, disk_size, used, free;
 
  blksize = buf.f_bsize;
  blocks = buf.f_blocks;
  freeblks = buf.f_bfree;
   
  disk_size = blocks * blksize;
  free = freeblks * blksize;
  used = disk_size - free;
 
//  printf("Disk usage : %lu \t Free space %lu\n", used, free/1024/1024/1024);} else {
//  printf("Couldn't get file system statistics\n");

  return (double)(free/1024/1024)/1024.0;
  }


}


int main(int argc, char **argv) {
  int i = 0;
  char* fname = "hubo-ach-log.log";
  while(argc > i) {
    if(strcmp(argv[i], "-f") == 0) {
        if( argc > (i+1)) {
            fname = argv[i+1];
        }
      }
    if(strcmp(argv[i], "-h") == 0) {

      printf("\n");
      printf("Usage: hubo-log -f fileName\n");
      printf("\tOptions:\n");
      printf("\t\t-h   help menu\n");
      printf("\t\t-f   log file name\n");
      printf("\n");
      return 0;
    }
      i++;
  }

   daemon(0,0);





    /* Open Ach Channel */
    int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME , NULL);
    assert( ACH_OK == r );

    r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME , NULL);
    assert( ACH_OK == r );

    // open to sim chan
    r = ach_open(&chan_hubo_to_sim, HUBO_CHAN_VIRTUAL_TO_SIM_NAME, NULL);
    assert( ACH_OK == r );



    /* Create initial structures to read and write from */
    hubo_ref_t H_ref;
    hubo_state_t H_state;
    hubo_virtual_t H_virtual;

    memset( &H_ref,   0, sizeof(H_ref));
    memset( &H_state, 0, sizeof(H_state));
    memset( &H_virtual, 0, sizeof(H_virtual));

    /* for size check */
    size_t fs;

    FILE *file;
    file = fopen(fname,"w");
    int fd = open(fname, O_WRONLY);
    ach_flush(&chan_hubo_to_sim);
    ach_flush(&chan_hubo_state);

    double free = getSpaceLeft(fname);
    printf("Free space %f\n\r", free); 

    int checkTime = HUBO_ACH_LOG_CHECK_TIME;  // check free space every x seconds
    int endi = (int)((double)checkTime*(1/(double)HUBO_LOOP_PERIOD));

    i = 0;
  while(1) {
/* 
   r = ach_get( &chan_hubo_to_sim, &H_virtual, sizeof(H_virtual), &fs, NULL, ACH_O_WAIT );
    if(ACH_OK != r) {
        assert( sizeof(H_virtual) == fs );
    }
*/
    /* Get the current feed-forward (state) */
    r = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_WAIT );
//    if(ACH_OK != r) {
//        assert( sizeof(H_state) == fs );
//    }


  if(ACH_OK == r) {
    write(fd, &H_state, sizeof(H_state));
    if(i < endi){
      i = i+1;
    }
    else{
      free = getSpaceLeft(fname);
      if( free < HUBO_ACH_LOG_GB_BREAK){
        printf("Low Disk Space: Logger Stopping\n\r"); 
        close(fd);
        break;
      }
      i = 0;
    }
  }

//    printf("wrote \n\r");


  }
}

