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
#define NUM_OF_SENSOR_PARAMETERS 7
#define NUM_OF_HOME_PARAMETERS 7

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
		if (NUM_OF_SENSOR_PARAMETERS == sscanf(buff, "%s%hu%hhu%hu%hhd%hhd%hhd",
			tp.name,
			&tp.can,
			&tp.active,
			&tp.boardNo,
            &tp.xsign,
            &tp.ysign,
            &tp.zsign	) ) // check that all values are found
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

int wait_on_state(ach_channel_t *chan, int secs)
{
    hubo_state_t state;
    size_t fs=0;
    struct timespec timeout;
    clock_gettime( ACH_DEFAULT_CLOCK, &timeout );
    timeout.tv_sec += secs;
    ach_status_t r = ach_get(chan, &state, sizeof(state), &fs, &timeout, ACH_O_LAST | ACH_O_WAIT);
    if( ACH_TIMEOUT == r )
        return -1;
    else if( ACH_OK==r || ACH_MISSED_FRAME==r )
        return 1;
    
    fprintf(stderr, "Unexpected result while waiting for hubo-state: %s\n", ach_result_to_string(r));
    return 0;
}


int loadHomingParams( const char *file_name )
{
    FILE *ptr_file;

    ach_channel_t cmd_chan;
    ach_status_t r = ach_open(&cmd_chan, HUBO_CHAN_BOARD_CMD_NAME, NULL);
    if( ACH_OK != r )
    {
        fprintf(stderr, "Failed to open %s!", HUBO_CHAN_BOARD_CMD_NAME);
        return -1;
    }

    ach_channel_t state_chan;
    r = ach_open(&state_chan, HUBO_CHAN_STATE_NAME, NULL);
    if( ACH_OK != r )
    {
        fprintf(stderr, "Failed to open %s!", HUBO_CHAN_STATE_NAME);
        return -1;
    }


    if( !(ptr_file=fopen(file_name, "r")) )
    {
        fprintf(stderr, "Unable to locate %s\n"
                " -- Check if the file exists and provide the full path!\n", file_name);
        return -1;
    }

    hubo_board_joint_param_t tempJP;
    hubo_board_cmd_t cmd;
    memset(&tempJP, 0, sizeof(tempJP));
    memset(&cmd, 0, sizeof(cmd));

    char* charPointer;
	char buff[1024];
	// read in each non-commented line of the config file corresponding to each joint
	while (fgets(buff, sizeof(buff), ptr_file) != NULL)
    {

		// set first occurrence of comment character, '#' to the
		// null character, '\0'.
		charPointer = strchr(buff, '#');
		if (NULL != charPointer) {
			*charPointer = '\0';
		}

		// check if a line is longer than the buffer, 'buff', and return -1 if so.
		if ( strlen(buff) == sizeof(buff)-1 ) {
			fprintf(stderr, "Hubo-Parser: Line length overflow in %s\n"
                            " -- One of the lines in the file is way too long\n", file_name);
			return -1; // parsing failed
		}

        char tempName[5];
		char tempType[5];
        // read in the buffered line from fgets, matching the following pattern
		// to get all the parameters for the joint on this line.
		if (NUM_OF_HOME_PARAMETERS == sscanf(buff, "%s%lf%lf%lf%hhu%hu%s",
            tempName,
            &tempJP.homeOffset,
            &tempJP.lowerLimit,
            &tempJP.upperLimit,
            &tempJP.searchDirection,
            &tempJP.searchLimit,
            tempType) ) // check that all values are found
        {
            // check to make sure jointName is valid
            size_t i;
            size_t jntNameCount = 0;
            size_t jntIndex;
            for (jntIndex = 0; jntIndex < HUBO_JOINT_COUNT; jntIndex++) {
                if (0 == strcmp(tempName, jointNames[jntIndex])) {
                    i = jntIndex;
                    jntNameCount = 1;
                    break;
                }
            }
            
            // if joint name is invalid print error and return -1
            if (jntNameCount != 1) {
                fprintf(stderr, "Joint name '%s' is incorrect in file '%s'\n"
                                " -- We are skipping this line!\n", tempName, file_name);
                continue;
			}
            
            if( 0 == strcmp(tempType, "raw") )
            {
                cmd.type = D_SET_HOME_PARAMS_RAW;
                cmd.joint = i;
                cmd.iValues[0] = tempJP.searchLimit;
                cmd.iValues[1] = tempJP.searchDirection;
                cmd.iValues[2] = (int)(tempJP.homeOffset);

                ach_put(&cmd_chan, &cmd, sizeof(cmd));
                wait_on_state(&state_chan, 5);

                cmd.type = D_SET_LOW_POS_LIM_RAW;
                cmd.joint = i;
                cmd.iValues[0] = (int32_t)(tempJP.lowerLimit);
                cmd.iValues[1] = 1;
                cmd.iValues[2] = 1;
                
                ach_put(&cmd_chan, &cmd, sizeof(cmd));
                wait_on_state(&state_chan, 5);
                
                cmd.type = D_SET_UPP_POS_LIM_RAW;
                cmd.iValues[0] = (int32_t)(tempJP.upperLimit);
                
                ach_put(&cmd_chan, &cmd, sizeof(cmd));
                wait_on_state(&state_chan, 5);
            }
            else if( 0 == strcmp(tempType, "rad") )
            {
                cmd.type = D_SET_HOME_PARAMS;
                cmd.joint = i;

                if(tempJP.searchDirection == 0)
                    cmd.param[0] = D_CLOCKWISE;
                else if(tempJP.searchDirection == 1)
                    cmd.param[0] = D_COUNTERCLOCKWISE;
                else
                {
                    fprintf(stderr, "Invalid value for search direction (%d) on joint '%s' in file '%s'\n"
                                    " -- Must be 0 (Clockwise) or 1 (Counter-Clockwise)\n"
                                    " -- We are skipping this line!\n",
                                    tempJP.searchDirection, tempName, file_name);
                    continue;
                }
                
                cmd.dValues[0] = tempJP.homeOffset;
                cmd.iValues[0] = tempJP.searchLimit;

                ach_put(&cmd_chan, &cmd, sizeof(cmd));
                wait_on_state(&state_chan, 5);
                
                cmd.type = D_SET_LOW_POS_LIM;
                cmd.joint = i;
                cmd.dValues[0] = tempJP.lowerLimit;
                cmd.param[0] = D_UPDATE;
                cmd.param[1] = D_ENABLE;
                
                ach_put(&cmd_chan, &cmd, sizeof(cmd));
                wait_on_state(&state_chan, 5);
                
                cmd.type = D_SET_UPP_POS_LIM;
                cmd.dValues[0] = tempJP.upperLimit;
                
                ach_put(&cmd_chan, &cmd, sizeof(cmd));
                wait_on_state(&state_chan, 5);
            }
            else
            {
                fprintf(stderr, "Unit type '%s' is invalid for joint '%s' in file '%s'\n"
                                " -- Change this to 'raw' for encoder units or 'rad' for radians!\n",
                                tempType, tempName, file_name);
            } // TODO: Add in degrees??

		}

	}

	fclose(ptr_file);	// close file stream

	return 0;	// return without errors
    

}

int saveHomingParams( const char *file_name, int type )
{
    FILE *ptr_file;

    ach_channel_t param_chan;
    ach_status_t r = ach_open(&param_chan, HUBO_CHAN_BOARD_PARAM_NAME, NULL);
    if( ACH_OK != r )
    {
        fprintf(stderr, "Failed to open %s!", HUBO_CHAN_BOARD_PARAM_NAME);
        return -1;
    }

    ach_channel_t state_chan;
    r = ach_open(&state_chan, HUBO_CHAN_STATE_NAME, NULL);
    if( ACH_OK != r )
    {
        fprintf(stderr, "Failed to open %s channel!", HUBO_CHAN_STATE_NAME);
        ach_close(&param_chan);
        return -1;
    }

    if( !(ptr_file=fopen(file_name, "w")) )
    {
        fprintf(stderr, "Unable to locate file %s\n"
                " -- Check if the file exists and provide the full path!\n", file_name);
        ach_close(&state_chan);
        ach_close(&param_chan);
        return -1;
    }
    
    hubo_board_param_t params;
    hubo_state_t state;
    memset(&params, 0, sizeof(params));
    memset(&state,  0, sizeof(state));

    size_t fs;
    struct timespec timeout;
    clock_gettime( ACH_DEFAULT_CLOCK, &timeout );
    timeout.tv_sec += 5;
    r = ach_get(&state_chan, &state, sizeof(state), &fs, &timeout, ACH_O_LAST | ACH_O_WAIT);
    
    if( ACH_TIMEOUT == r )
    {
        fprintf(stderr, "saveHomeParams( ~ ) could not find anything on the state channel!\n"
                        " -- Ach result: %s\n"
                        " -- Make sure hubo-daemon is active\n", ach_result_to_string(r));
        ach_close(&state_chan);
        ach_close(&param_chan);
        fclose(ptr_file);
        return -1;
    }

    r = ach_get(&param_chan, &params, sizeof(params), &fs, &timeout, ACH_O_LAST);
    
    if( !(ACH_OK == r || ACH_MISSED_FRAME == r) )
    {
        fprintf(stderr, "saveHomeParams( ~ ) could not find anything on the %s channel!\n"
                        " -- Ach result: %s\n"
                        " -- Try rebooting hubo-ach\n", HUBO_CHAN_BOARD_PARAM_NAME, ach_result_to_string(r));
        ach_close(&state_chan);
        ach_close(&param_chan);
        fclose(ptr_file);
        return -1;
    }

    // Print a header
    fprintf(ptr_file, "# File name: %s\n", file_name);
    fprintf(ptr_file, "# This table was automatically generated by the saveHomingParams( ~ ) function\n"
                      "# provided by libhuboparams. The values in this table are based on the homing\n"
                      "# parameters on the hardware at the time that the function was run.\n");

    if( type==0 )
        fprintf(ptr_file, "\n# -- Units in this table are in terms of encoder values (raw)\n");
    else if( type==1 )
        fprintf(ptr_file, "\n# -- Units in this table are in terms of radians (rad)\n");
    else
    {
        fprintf(ptr_file, "\n# -- Invalid unit type requested! (%d)\n"
                          "# Must be raw encoder units (0) or radians (1)\n", type);
        ach_close(&state_chan);
        ach_close(&param_chan);
        fclose(ptr_file);
        return -1;
    }

    fprintf(ptr_file, "\n");
    fprintf(ptr_file,  "JointName   HomeOffset  LowerLimit  UpperLimit  SearchDirection SearchLimit Type\n");

    size_t i=0;
    for(i=0; i<HUBO_JOINT_COUNT; i++)
    {
        if(state.joint[i].active == 1 && params.joint[i].confidence == 1)
        {
            if( type==0 ) // Raw
            {
                fprintf(ptr_file,
    //                      The minus sign pads on the right instead of left
                           "%s         "        // Joint Name
                           "%-12d"         // Home Offset
                           "%-12d"         // Lower Limit
                           "%-12d"         // Upper Limit
                           "%-16d"  // Search Direction
                           "%-12d"      // Search Limit
                           "raw \n",
                            jointNames[i],
                            params.joint[i].homeOffsetRaw,
                            params.joint[i].lowerLimitRaw,
                            params.joint[i].upperLimitRaw,
                            params.joint[i].searchDirection,
                            params.joint[i].searchLimit);
            }
            else if( type==1 ) // Radian
            {
                fprintf(ptr_file,
    //                      The minus sign pads on the right instead of left
                           "%s         "        // Joint Name
                           "%-2.4f     "         // Home Offset
                           "%-2.4f     "         // Lower Limit
                           "%-2.4f     "         // Upper Limit
                           "%-3d             "  // Search Direction
                           "%-3d         "      // Search Limit
                           "raw \n",
                            jointNames[i],
                            params.joint[i].homeOffset,
                            params.joint[i].lowerLimit,
                            params.joint[i].upperLimit,
                            params.joint[i].searchDirection,
                            params.joint[i].searchLimit);
            }
        }
    }

    ach_close(&state_chan);
    ach_close(&param_chan);
    fclose(ptr_file);
    return 0;
}

int setJointParams(hubo_param_t *H_param, struct hubo_state *H_state) {
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
	struct hubo_joint_state s;	//hubo_joint_state struct for file parsing

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
		if (NUM_OF_JOINT_PARAMETERS == sscanf(buff, "%s%hu%u%hu%hu%hu%hu%hhd%s%hhu%hhu%hhu%hhu",
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
			size_t jntIndex;
			for (jntIndex = 0; jntIndex < HUBO_JOINT_COUNT; jntIndex++) {
				if (0 == strcmp(tp.name, jointNames[jntIndex])) {
					i = jntIndex;
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

