/* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
/*
Copyright (c) 2013, Daniel M. Lofaro
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

#include <unistd.h>

int loop();

/* Ach Channel IDs */
ach_channel_t chan_hubo_ref;      // Feed-Forward (Reference)
ach_channel_t chan_hubo_ref_multi;    //

int main(int argc, char **argv) {

    // Daemonize
    hubo_daemonize("hubo-multi-chan", 49);

    loop();

}

int loop(){
    int i = 0;

    /* for size check */
    size_t fs;

    /* Open Ach Channel */
    int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME , NULL);
    hubo_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_ref_multi, HUBO_CHAN_MULTI_CHAN_NAME , NULL);
    hubo_assert( ACH_OK == r, __LINE__ );
    ach_flush(&chan_hubo_ref_multi);


    /* Create initial structures to read and write from */
    struct hubo_ref H_ref;
    struct hubo_ref H_ref_multi;
    memset( &H_ref,   0, sizeof(H_ref));
    memset( &H_ref_multi, 0, sizeof(H_ref_multi));

    r = ach_get( &chan_hubo_ref_multi, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );  
    if(ACH_OK != r) { hubo_assert( sizeof(H_ref) == fs, __LINE__ );}

    memcpy( &H_ref_multi, &H_ref, sizeof(H_ref_multi));

    ach_put( &chan_hubo_ref, &H_ref, sizeof(H_ref));


    usleep(2);
    while(1){
        /* Wait for new feedforward  */
        r = ach_get( &chan_hubo_ref, &H_ref_multi, sizeof(H_ref_multi), &fs, NULL, ACH_O_WAIT );  
        if(ACH_OK != r) { hubo_assert( sizeof(H_ref_multi) == fs, __LINE__ );}

        for( i = 0; i < HUBO_JOINT_COUNT ; i++ ){
            if(HUBO_JOINT_REF_ACTIVE == H_ref_multi.active[i]){
                H_ref.ref[i] = H_ref_multi.ref[i];
                H_ref.mode[i] = H_ref_multi.mode[i];
            }
        }

        /* Write to the feed-forward channel */
        ach_put( &chan_hubo_ref_multi, &H_ref, sizeof(H_ref));
//        usleep(10000);
    }
}

