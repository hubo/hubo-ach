#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include "hubo.h"
#include "hubo-jointparams.h"

int setJointParams(struct hubo_param *H) {
        FILE *ptr_file;

        // open file and if fails, return 1
        if (!(ptr_file=fopen("hubo-config.txt", "r")))
                return 1;

        struct hubo_joint_param tp;                     //instantiate hubo_jubo_param struct
	struct jmcDriver tp2;
	memset(&tp,  0, sizeof(tp));
	memset(&tp2, 0, sizeof(tp2));
	size_t x;
	size_t y;
	for(x = 0; x < HUBO_JMC_COUNT; x++) {
		for(y = 0; y < sizeof(&H->driver[x].jmc); y++) {
			H->driver[x].jmc[y] = 0;
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

	
	size_t i;
	size_t jntNameCount = 0;
	size_t jmcNameCount = 0;
	char jmc[6];
        char buff[100];
        // read in each non-commented line of the config file corresponding to each joint
        while (fgets(buff, sizeof(buff), ptr_file) != NULL) {
                if (buff[0] != '#' && buff[0] != '\n') {
                       // printf("buff: %s\n", buff);
                        sscanf(buff, "%s%hu%u%hu%hu%hu%hu%hhu%s%hhu%hhu%hhu%hhu\n",
                        tp.name,
                        &tp.motNo,
                        &tp.refEnc,
                        &tp.drive,
                        &tp.driven,
                        &tp.harmonic,
                        &tp.enc,
                        &tp.dir,
			jmc,
                        &tp.can,
                        &tp.active,
                        &tp.numMot,
                        &tp.zeroed);
			
			size_t z;
			for(z = 0; z < sizeof(jointNamesStrings); z++) {
				if (0 == strcmp(tp.name, jointNamesStrings[z])) {
					i = jointNamesShorts[z];
					jntNameCount = 1;
					break;
				}
			}
			if (jntNameCount != 1) {
				printf("joint name '%s' is incorrect\n", tp.name);
				exit(EXIT_FAILURE);
			}

			size_t j;
			for(j = 0; j < sizeof(jmcNames); j++) {
				if (0 == strcmp(jmc, jmcNames[j])) {
					tp.jmc = jmcNumbers[j];
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
		}
        }
        // close file stream
        fclose(ptr_file);
 
     	return 0;
}
