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
static char *fileLocation = "/etc/hubo-daemon/joint.table";

// ach channels
ach_channel_t chan_hubo_ref;
ach_channel_t chan_hubo_init_cmd;


//Hack way to set up sensor until a table can be made
void setupSensorDefaults(struct hubo_param* H) {

	sprintf(H->sensor[HUBO_FT_R_FOOT].name , "%s", "FT0");
	sprintf(H->sensor[HUBO_FT_L_FOOT].name , "%s", "FT1" );
	sprintf(H->sensor[HUBO_FT_R_HAND].name , "%s", "FT2" );
	sprintf(H->sensor[HUBO_FT_L_HAND].name , "%s", "FT3" );
	sprintf(H->sensor[HUBO_IMU0].name , "%s", "IMU0" );
	sprintf(H->sensor[HUBO_IMU1].name , "%s", "IMU1" );
	sprintf(H->sensor[HUBO_IMU2].name , "%s", "IMU2" );

	printf("Sensor 0 named %s\n",H->sensor[0].name);

	H->sensor[HUBO_FT_R_FOOT].boardNo = 1;
	H->sensor[HUBO_FT_L_FOOT].boardNo = 2;
	H->sensor[HUBO_FT_R_HAND].boardNo = 6;
	H->sensor[HUBO_FT_L_HAND].boardNo = 7;
	H->sensor[HUBO_IMU0].boardNo = 3;
	H->sensor[HUBO_IMU1].boardNo = 4;
	H->sensor[HUBO_IMU2].boardNo = 5;


	H->sensor[HUBO_FT_R_FOOT].can = 0;
	H->sensor[HUBO_FT_L_FOOT].can = 0;
	H->sensor[HUBO_FT_R_HAND].can = 1;
	H->sensor[HUBO_FT_L_HAND].can = 1;
	H->sensor[HUBO_IMU0].can = 0;
	H->sensor[HUBO_IMU1].can = 0;
	H->sensor[HUBO_IMU2].can = 0;

	H->sensor[HUBO_FT_R_FOOT].sensNo = HUBO_FT_R_FOOT;
	H->sensor[HUBO_FT_L_FOOT].sensNo = HUBO_FT_L_FOOT;
	H->sensor[HUBO_FT_R_HAND].sensNo = HUBO_FT_R_HAND;
	H->sensor[HUBO_FT_L_HAND].sensNo = HUBO_FT_L_HAND;
	H->sensor[HUBO_IMU0].sensNo = HUBO_IMU0;
	H->sensor[HUBO_IMU1].sensNo = HUBO_IMU1;
	H->sensor[HUBO_IMU2].sensNo = HUBO_IMU2;

}

int setJointParams(struct hubo_param *H_param, struct hubo_state *H_state) {
//	char *envVar = getenv("HUBO_JOINT_TABLE");
//	printf("%s\n", envVar);
//	if(strcmp(envVar, fileLocation) != 0) exit(EXIT_FAILURE);

	FILE *ptr_file;

	// open file for read access and if it fails, return -1
	if (!(ptr_file=fopen(fileLocation, "r")))
		return -1;

	// instantiate stucts for getting values from joint.table
	// file and copying them to 
	struct hubo_joint_param tp;	//hubo_jubo_param struct for file parsing
	struct jmcDriver tp2;		//jmcDriver struct member for file parsing
	struct hubo_joint_state s;	//hubo_joint_state struct for file parsing

	// initialize all structs with zeros
	memset(&tp,	 0, sizeof(tp));
	memset(&tp2,	 0, sizeof(tp2));
	memset(&s,	 0, sizeof(s));
	size_t i;
	size_t j;

	// inialize jmcDriver struct's jmc numbers with zeros.
	// these are the motors on each motor driver.
	for (i = 0; i < HUBO_JMC_COUNT; i++) {
		for (j = 0; j < sizeof(&H_param->driver[i].jmc); j++) {
			H_param->driver[i].jmc[j] = 0;
		}
	}

	// array of joint name values from header file hubo.h
	uint16_t jointNameValues[] = 
			{WST, NKY, NK1, NK2,
			LSP, LSR, LSY, LEB, LWY, LWR, LWP,
			RSP, RSR, RSY, REB, RWY, RWR, RWP,
			LHY, LHR, LHP, LKN, LAP, LAR,
			RHY, RHR, RHP, RKN, RAP, RAR,
			RF1, RF2, RF3, RF4, RF5,
			LF1, LF2, LF3, LF4, LF5};

	// array of joint name strings (total of 40)
	char *jointNameStrings[] = 
			{"WST", "NKY", "NK1", "NK2", 
			 "LSP", "LSR", "LSY", "LEB", "LWY", "LWR", "LWP",
			 "RSP", "RSR", "RSY", "REB", "RWY", "RWR", "RWP",
			 "LHY", "LHR", "LHP", "LKN", "LAP", "LAR",
			 "RHY", "RHR", "RHP", "RKN", "RAP", "RAR",
			 "RF1", "RF2", "RF3", "RF4", "RF5",
			 "LF1", "LF2", "LF3", "LF4", "LF5"};

	// array of jmc name values from header file canId.h
	uint8_t jmcNumbers[] = {JMC0, JMC1, JMC2, JMC3, JMC4, JMC5,
				JMC6, JMC7, JMC8, JMC9, JMC10, JMC11,
				EJMC0, EJMC1, EJMC2, EJMC3, EJMC4, EJMC5};

	// array of jmc name strings (total of 18)
	char *jmcNames[] = {"JMC0", "JMC1", "JMC2", "JMC3", "JMC4", "JMC5",
			    "JMC6", "JMC7", "JMC8", "JMC9", "JMC10", "JMC11",
			    "EJMC0", "EJMC1", "EJMC2", "EJMC3", "EJMC4", "EJMC5"};

	char *charPointer;
	size_t jntNameCount = 0;
	size_t jmcNameCount = 0;
	char jmc[6];
	char buff[1024];

	// read in each non-commented line of the config file corresponding to each joint
	while (fgets(buff, sizeof(buff), ptr_file) != NULL) {

			// set first occurrence of comment character, '#' to the
			// null character, '\0'.
			charPointer = strchr(buff, '#');
			if (NULL != charPointer) {
				*charPointer = '\0';
			}

			// check if a line is longer than the buffer, 'buff', and return -1 if so.
			if ( strlen(buff) == sizeof(buff)-1 ) {
				fprintf(stderr, "Hubo-Parser: Line length overflow");
				return -1; // parsing failed
			}

			// read in the buffered line from fgets, matching the following pattern
			// to get all the parameters for the joint on this line.
			if (13 == sscanf(buff, "%s%hu%u%hu%hu%hu%hu%hhu%s%hhu%hhu%hhu%hhu",
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
				&s.zeroed) ) // check that all values are found
			{
				
				// check to make sure jointName is valid
				size_t x;
				for (x = 0; x < sizeof(jointNameStrings)/sizeof(jointNameStrings[0]); x++) {
					if (0 == strcmp(tp.name, jointNameStrings[x])) {
						i = jointNameValues[x];
						jntNameCount = 1;
						break;
					}
				}

				// if joint name is invalid print error and return -1
				if (jntNameCount != 1) {
					fprintf(stderr, "joint name '%s' is incorrect\n", tp.name);
					return -1; // parsing failed
				}

				// check to make sure jmc name is valid
				size_t y;
				for(y = 0; y < sizeof(jmcNames)/sizeof(jmcNames[0]); y++) {
					if (0 == strcmp(jmc, jmcNames[y])) {
						tp.jmc = jmcNumbers[y];
						jmcNameCount = 1;
						break;
					}	
				}

				// if jmc name is invalid, print error and return -1
				if (jmcNameCount != 1) {
					fprintf(stderr, "jmc name '%s' is incorrect\n", jmc);
					return -1; // parsing failed
				}

				tp.jntNo = i;		// define i to be the joint number
				tp2.jmc[tp.motNo] = i;	// set jmc driver number	

				// copy contents (all member values) of tp into H_param.joint 
				// substruct which will populate its member variables
				memcpy(&(H_param->joint[i]), &tp, sizeof(tp));
				// copy contents of tp.jmc into H_param structs driver substruct
				memcpy(&(H_param->driver[tp.jmc].jmc[tp.motNo]), &tp2.jmc[tp.motNo], sizeof(tp2.jmc[tp.motNo]));
				// copy contents of s into H_state (initializing active and zeroed members)
				memcpy(&(H_state->joint[i]), &s, sizeof(s));
			}
        }

        fclose(ptr_file);	// close file stream

	// print the paramter values in H_param for each joint
/*	printf("jntNo\tname\tmotNo\trefEnc\tdrive\tdriven\tharm\tenc\tdir\tjmc\tcan\tnumMot\n"); 
	for (i = 0; i < HUBO_JOINT_COUNT; i++) {
		printf ("%hu\t%s\t%hu\t%u\t%hu\t%hu\t%hu\t%hu\t%hhu\t%hu\t%hhu\t%hhu\n",
		H_param->joint[i].jntNo,
		H_param->joint[i].name,
		H_param->joint[i].motNo,
		H_param->joint[i].refEnc,
		H_param->joint[i].drive,
		H_param->joint[i].driven,
		H_param->joint[i].harmonic,
		H_param->joint[i].enc,
		H_param->joint[i].dir,
		H_param->joint[i].jmc,
		H_param->joint[i].can,
		H_param->joint[i].numMot);
        }
*/	        
	// print values of driver jmc motor numbers in H_param
/*	for (i = 0; i < HUBO_JMC_COUNT; i++) {
		printf("%lu\t%hhu\t%hhu\t%hhu\t%hhu\t%hhu\n",
			i,
			H_param->driver[i].jmc[0],
			H_param->driver[i].jmc[1],
			H_param->driver[i].jmc[2],
			H_param->driver[i].jmc[3],
			H_param->driver[i].jmc[4]);
        }
*/
/*	// print values saved in H_state.joint[i].active and H_state.joint[i].zeroed
	for (i = 0; i < HUBO_JOINT_COUNT; i++) {
		printf("%s\t%hhu\t%hhu\n", H->joint[i].name, H_state->joint[i].active, H_state->joint[i].zeroed);
	} 
*/	
	setupSensorDefaults(H_param);
	return 0;	// return without errors
}

void setPosZeros() {
        // open ach channel
//        int r = ach_open(&chan_num, "hubo", NULL);
//        assert( ACH_OK == r );

	// open hubo reference
        int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME, NULL);
        assert( ACH_OK == r );

        struct hubo_ref H_ref;
        memset( &H_ref,   0, sizeof(H_ref));
        size_t fs = 0;

        r = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
        assert( sizeof(H_ref) == fs );

        size_t i;
        for( i = 0; i < HUBO_JOINT_COUNT; i++) {
                H_ref.ref[i] = 0.0;
        }
        ach_put(&chan_hubo_ref, &H_ref, sizeof(H_ref));
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
