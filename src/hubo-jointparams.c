// standard
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>

// for hubo
#include "hubo.h"
#include "hubo/canID.h"
#include "hubo-daemonID.h"
#include "hubo-daemon.h"
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
#include "config.h"

//set number of parameters per joint in the parameters file
#define	NUM_OF_JOINT_PARAMETERS 13
#define NUM_OF_SENSOR_PARAMETERS 4

//set file location
static char *jointFileLocation = "/etc/" PACKAGE_NAME "/joint.table";
static char *sensorFileLocation = "/etc/" PACKAGE_NAME "/sensor.table";




int setSensorDefaults( hubo_param_t *h ) {
    
	FILE *ptr_file;

	// open file for read access and if it fails, return -1
	if (!(ptr_file=fopen(sensorFileLocation, "r")))
    {
        fprintf(stderr, "Unable to locate %s\n -- Try reinstalling or reconfiguring!\n",sensorFileLocation);
		return -1;
    }
	// instantiate stucts for getting values from joint.table
	// file and copying them to
	hubo_sensor_param_t tp;	//hubo_jubo_param struct for file parsing

	// initialize all structs with zeros
	memset(&tp,      0, sizeof(tp));


    hubo_sensor_index_t index[] = { HUBO_FT_R_FOOT, HUBO_FT_L_FOOT, HUBO_FT_R_HAND, HUBO_FT_L_HAND, HUBO_IMU0, HUBO_IMU1, HUBO_IMU2 };

    char *sensorNameStrings[] =
            {"RFFT", "LFFT", "RHFT", "LHFT",
             "IMU0", "IMU1", "IMU2" };

	char *charPointer;
	char buff[1024];
    size_t sensorNameCount = 0;
    size_t i;

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
			fprintf(stderr, "Hubo-Parser: Line length overflow in sensor.table");
			return -1; // parsing failed
		}

		// read in the buffered line from fgets, matching the following pattern
		// to get all the parameters for the joint on this line.
		if (NUM_OF_SENSOR_PARAMETERS == sscanf(buff, "%s%hu%hhu%hu",
			tp.name,
			&tp.can,
			&tp.active,
			&tp.boardNo	) ) // check that all values are found
		{

			// check to make sure jointName is valid
			size_t x;
			for (x = 0; x < sizeof(sensorNameStrings)/sizeof(sensorNameStrings[0]); x++) {
				if (0 == strcmp(tp.name, sensorNameStrings[x])) {
					i = index[x];
					sensorNameCount = 1;
					break;
				}
			}

			// if joint name is invalid print error and return -1
			if (sensorNameCount != 1) {
				fprintf(stderr, "joint name '%s' is incorrect\n", tp.name);
				return -1; // parsing failed
			}

			tp.sensNo = i;		// define i to be the sensor number

			// copy contents (all member values) of tp into H_param.joint 
			// substruct which will populate its member variables
			memcpy(&(h->sensor[i]), &tp, sizeof(tp));
		}
	}

	fclose(ptr_file);	// close file stream

	return 0;	// return without errors
}



int setJointParams(hubo_param_t *H_param, hubo_state_t *H_state) {
//	char *envVar = getenv("HUBO_JOINT_TABLE");
//	printf("%s\n", envVar);
//	if(strcmp(envVar, jointFileLocation) != 0) exit(EXIT_FAILURE);

	FILE *ptr_file;

	// open file for read access and if it fails, return -1
	if (!(ptr_file=fopen(jointFileLocation, "r")))
    {
        fprintf(stderr, "Unable to locate %s\n -- Try reinstalling or reconfiguring!\n",jointFileLocation);
		return -1;
    }
	// instantiate stucts for getting values from joint.table
	// file and copying them to
	hubo_joint_param_t tp;	//hubo_jubo_param struct for file parsing
	hubo_jmc_param_t tp2;	//jmcDriver struct member for file parsing
	hubo_joint_state_t s;	//hubo_joint_state struct for file parsing

	// initialize all structs with zeros
	memset(&tp,      0, sizeof(tp));
	memset(&tp2,     0, sizeof(tp2));
	memset(&s,       0, sizeof(s));
	size_t i;
	size_t j;

	// inialize jmcDriver struct's jmc numbers with zeros.
	// these are the motors on each motor driver.
	for (i = 0; i < HUBO_JMC_COUNT; i++) {
		for (j = 0; j < sizeof(&H_param->driver[i].joints); j++) {
			H_param->driver[i].joints[j] = 0;
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
			fprintf(stderr, "Hubo-Parser: Line length overflow in joint.table");
			return -1; // parsing failed
		}

		// read in the buffered line from fgets, matching the following pattern
		// to get all the parameters for the joint on this line.
		if (NUM_OF_JOINT_PARAMETERS == sscanf(buff, "%s%hu%u%hu%hu%hu%hu%hhu%s%hhu%hhu%hhu%hhu",
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
			tp2.joints[tp.motNo] = i;	// set jmc driver number	

			// copy contents (all member values) of tp into H_param.joint 
			// substruct which will populate its member variables
			memcpy(&(H_param->joint[i]), &tp, sizeof(tp));
			// copy contents of tp.jmc into H_param structs driver substruct
			memcpy(&(H_param->driver[tp.jmc].joints[tp.motNo]), &tp2.joints[tp.motNo], sizeof(tp2.joints[tp.motNo]));
			// copy contents of s into H_state (initializing active and zeroed members)
			memcpy(&(H_state->joint[i]), &s, sizeof(s));
		}
	}

	fclose(ptr_file);	// close file stream

	return 0;	// return without errors
}

