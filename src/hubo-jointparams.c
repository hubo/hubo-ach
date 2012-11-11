// standard
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>

// for hubo
#include "hubo.h"
#include "hubo-jointparams.h"

// for ach
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

//set number of parameters per joint in the parameters file
#define	NUM_OF_PARAMETERS 13

//set file location
char *fileLocation = "/etc/hubo/jointtab";

// ach channels
ach_channel_t chan_hubo_ref;      // hubo-ach
ach_channel_t chan_hubo_init_cmd; // hubo-ach-console
ach_channel_t chan_hubo_state;    // hubo-ach-state


int setJointParams(struct hubo_param *H) {
//	char *envVar = getenv("HUBO_JOINT_TABLE");
//	printf("%s\n", envVar);
//	if(strcmp(envVar, fileLocation) != 0) exit(EXIT_FAILURE);

        FILE *ptr_file;

        // open file and if fails, return 1
        if (!(ptr_file=fopen(fileLocation, "r")))
                return 1;

        struct hubo_joint_param tp;                     //instantiate hubo_jubo_param struct
	struct jmcDriver tp2;
	struct hubo_joint_state s;
	struct hubo_state H_state;
	memset(&tp,	 0, sizeof(tp));
	memset(&tp2,	 0, sizeof(tp2));
	memset(&s,	 0, sizeof(s));
	memset(&H_state, 0, sizeof(H_state));
	size_t i;
	size_t j;

	// open hubo state channel
        int r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME, NULL);
        assert( ACH_OK == r );

	size_t fs;
	r = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
        if(ACH_OK != r) {printf("State r = %s\n",ach_result_to_string(r));}
        assert( sizeof(H_state) == fs );


	for(i = 0; i < HUBO_JMC_COUNT; i++) {
		for(j = 0; j < sizeof(&H->driver[i].jmc); j++) {
			H->driver[i].jmc[j] = 0;
		}
	}

	uint16_t jointNamesShorts[] = 
			{WST, NKY, NK1, NK2,
			LSP, LSR, LSY, LEB, LWY, LWR, LWP,
			RSP, RSR, RSY, REB, RWY, RWR, RWP,
			LHY, LHR, LHP, LKN, LAP, LAR,
			RHY, RHR, RHP, RKN, RAP, RAR,
			RF1, RF2, RF3, RF4, RF5,
			LF1, LF2, LF3, LF4, LF5};

	char *jointNamesStrings[] = 
			{"WST", "NKY", "NK1", "NK2", 
			 "LSP", "LSR", "LSY", "LEB", "LWY", "LWR", "LWP",
			 "RSP", "RSR", "RSY", "REB", "RWY", "RWR", "RWP",
			 "LHY", "LHR", "LHP", "LKN", "LAP", "LAR",
			 "RHY", "RHR", "RHP", "RKN", "RAP", "RAR",
			 "RF1", "RF2", "RF3", "RF4", "RF5",
			 "LF1", "LF2", "LF3", "LF4", "LF5"};
	
	uint8_t jmcNumbers[] = {JMC0, JMC1, JMC2, JMC3, JMC4, JMC5,
				JMC6, JMC7, JMC8, JMC9, JMC10, JMC11,
				EJMC0, EJMC1, EJMC2, EJMC3, EJMC4, EJMC5};

	char *jmcNames[] = {"JMC0", "JMC1", "JMC2", "JMC3", "JMC4", "JMC5",
			    "JMC6", "JMC7", "JMC8", "JMC9", "JMC10", "JMC11",
			    "EJMC0", "EJMC1", "EJMC2", "EJMC3", "EJMC4", "EJMC5"};

	size_t numOfArgs;
	size_t jntNameCount = 0;
	size_t jmcNameCount = 0;
	char jmc[6];
        char buff[100];
        // read in each non-commented line of the config file corresponding to each joint
        while (fgets(buff, sizeof(buff), ptr_file) != NULL) {
                if (strchr(buff, '#') == NULL) {
                       // printf("buff: %s\n", buff);
                       if(numOfArgs = sscanf(buff, "%s%hu%u%hu%hu%hu%hu%hhu%s%hhu%hhu%hhu%hhu\n",
				tp.name,
				&tp.motNo,
				&tp.refEnc,
				&tp.drive,
				&tp.driven,
				&tp.harmonic,
				&tp.enc,
				&tp.dir,
				jmc,
				&s.active,
				&tp.can,
				&tp.numMot,
				&s.zeroed) == NUM_OF_PARAMETERS) {

				size_t x;
				for(x = 0; x < sizeof(jointNamesStrings); x++) {
					if (0 == strcmp(tp.name, jointNamesStrings[x])) {
					i = jointNamesShorts[x];
					jntNameCount = 1;
					break;
					}
				}
			
				if (jntNameCount != 1) {
					printf("joint name '%s' is incorrect\n", tp.name);
					exit(EXIT_FAILURE);
				}

				size_t y;
				for(y = 0; y < sizeof(jmcNames); y++) {
					if (0 == strcmp(jmc, jmcNames[y])) {
						tp.jmc = jmcNumbers[y];
						jmcNameCount = 1;
						break;
					}	
				}
			
				if (jmcNameCount != 1) {
					printf("jmc name '%s' is incorrect\n", jmc);
					exit(EXIT_FAILURE);
				}

				// define i to be the joint number
				tp.jntNo = i;
				// set jmc driver number	
				tp2.jmc[tp.motNo] = i;
				
				//copy contents (all member values) of tp into H.joint 
				//substruct which will populate its member variables
				memcpy(&(H->joint[i]), &tp, sizeof(tp));
				memcpy(&(H->driver[tp.jmc].jmc[tp.motNo]), &tp2.jmc[tp.motNo], sizeof(tp2.jmc[tp.motNo]));
				memcpy(&(H_state.joint[i]), &s, sizeof(s));
			}
			else {
				printf("number of arguments matched: %lu\n", numOfArgs);
				printf("malformed line in parameters file: %s\n", buff);
				exit(EXIT_FAILURE);
			} 
		}
        }
        // close file stream
        fclose(ptr_file);

/*	for (i = 0; i < HUBO_JOINT_COUNT; i++) {
		printf ("%hu\t%s\t%hu\t%u\t%hu\t%hu\t%hu\t%hu\t%hhu\t%hu\t%hhu\t%hhu\n",
		H->joint[i].jntNo,
		H->joint[i].name,
		H->joint[i].motNo,
		H->joint[i].refEnc,
		H->joint[i].drive,
		H->joint[i].driven,
		H->joint[i].harmonic,
		H->joint[i].enc,
		H->joint[i].dir,
		H->joint[i].jmc,
		H->joint[i].can,
		H->joint[i].numMot);
        }
*/	        
/*	for (i = 0; i < HUBO_JMC_COUNT; i++) {
		printf("%lu\t%hhu\t%hhu\t%hhu\t%hhu\t%hhu\n",
			i,
			H->driver[i].jmc[0],
			H->driver[i].jmc[1],
			H->driver[i].jmc[2],
			H->driver[i].jmc[3],
			H->driver[i].jmc[4]);
        }
*/
	// print values saved in H_state.joint[i].active and H_state.joint[i].zeroed
/*	for (i = 0; i < HUBO_JOINT_COUNT; i++) {
		printf("%s\t%hhu\t%hhu\n", H->joint[i].name, H_state.joint[i].active, H_state.joint[i].zeroed);
	} 
*/	
	ach_put(&chan_hubo_state, &H_state, sizeof(H_state));

	return 0;
}

void setPosZeros() {
        // open ach channel
//        int r = ach_open(&chan_num, "hubo", NULL);
//        assert( ACH_OK == r );
	
	// open hubo reference
        int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME, NULL);
        assert( ACH_OK == r );

        struct hubo_ref H;
        memset( &H,   0, sizeof(H));
        size_t fs = 0;

        r = ach_get( &chan_hubo_ref, &H, sizeof(H), &fs, NULL, ACH_O_LAST );
        assert( sizeof(H) == fs );

        size_t i;
        for( i = 0; i < HUBO_JOINT_COUNT; i++) {
                H.ref[i] = 0.0;
        }
        ach_put(&chan_hubo_ref, &H, sizeof(H));
}

void setConsoleFlags() {
	// initilize control channel
        int r = ach_open(&chan_hubo_init_cmd, HUBO_CHAN_INIT_CMD_NAME, NULL);
        assert( ACH_OK == r );
	
        struct hubo_init_cmd C;
        memset( &C,   0, sizeof(C));

        size_t fs =0;
        r = ach_get( &chan_hubo_init_cmd, &C, sizeof(C), &fs, NULL, ACH_O_LAST );
        assert( sizeof(C) == fs );
        int i = 0;
        for( i = 0; i < HUBO_JOINT_COUNT; i++ ) {
                C.cmd[i] = 0;
                C.val[i] = 0;
        }
        r = ach_put(&chan_hubo_init_cmd, &C, sizeof(C));
}
