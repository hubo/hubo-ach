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

/* Ach Channel IDs */
ach_channel_t chan_hubo_ref;      // Feed-Forward (Reference)
ach_channel_t chan_hubo_ref_multi;    //

int main(int argc, char **argv) {
    int i = 0;

    /* Open Ach Channel */
    int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME , NULL);
    assert( ACH_OK == r );

    r = ach_open(&chan_hubo_ref_multi, HUBO_CHAN_MULTI_CHAN_NAME , NULL);
    assert( ACH_OK == r );
    ach_flush(&chan_hubo_ref_multi);


    /* Create initial structures to read and write from */
    struct hubo_ref H_ref;
    struct hubo_ref H_ref_multi;
    memset( &H_ref,   0, sizeof(H_ref));
    memset( &H_ref_multi, 0, sizeof(H_ref_multi));

    /* for size check */
    size_t fs;

    for( i = 0; i < 5; i++){
        H_ref_multi.ref[WST] = (double)i;
        ach_put( &chan_hubo_ref_multi, &H_ref_multi, sizeof(H_ref_multi));
    }


    usleep(2);
    while(1){
        /* Wait for new feedforward  */
        r = ach_get( &chan_hubo_ref_multi, &H_ref_multi, sizeof(H_ref_multi), &fs, NULL, ACH_O_WAIT );  
        if(ACH_OK != r) { assert( sizeof(H_ref_multi) == fs );}
/*        if ( r == ACH_STALE_FRAMES ) {
            printf("Wait ----- \n\r");
            r = ach_get( &chan_hubo_ref_multi, &H_ref_multi, sizeof(H_ref_multi), &fs, NULL, ACH_O_WAIT );  
            if(ACH_OK != r) { assert( sizeof(H_ref_multi) == fs );}
        }
        else{
          //  r = ach_get( &chan_hubo_ref_multi, &H_ref_multi, sizeof(H_ref_multi), &fs, NULL, ACH_O_LAST );
//            r = ach_get( &chan_hubo_ref_multi, &H_ref_multi, sizeof(H_ref_multi), &fs, NULL, 0 );
            r = ach_get( &chan_hubo_ref_multi, &H_ref_multi, sizeof(H_ref_multi), &fs, NULL, ACH_O_WAIT );  
            if(ACH_OK != r) { assert( sizeof(H_ref_multi) == fs );}
	}
*/

        printf("WST = %f \n\r",H_ref_multi.ref[WST]);
        for( i = 0; i < HUBO_JOINT_COUNT ; i++ ){
            if(HUBO_JOINT_REF_ACTIVE == H_ref_multi.active[i]){
                H_ref.ref[i] = H_ref_multi.ref[i];
            }
        }

        /* Write to the feed-forward channel */
        ach_put( &chan_hubo_ref, &H_ref, sizeof(H_ref));
        usleep(10000);
    }
}

