/*
Copyright (c) 2012, Daniel M. Lofaro
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
#pragma GCC diagnostic ignored "-Wwrite-strings"
#include <stdio.h>
#include <stdlib.h>
#include <readline/readline.h>
#include <readline/history.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hubo.h"
// this is because this header file
// refers to a .c file not a .cpp file
extern "C" {
#include "hubo-jointparams.h"
}

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


#include <iostream>
#include <sstream>
#include <string>
using namespace std;

// ach message type
//typedef struct hubo h[1];

// ach channels
ach_channel_t chan_hubo_ref;      // hubo-ach
ach_channel_t chan_hubo_board_cmd; // hubo-ach-console
ach_channel_t chan_hubo_state;    // hubo-ach-state



static char** my_completion(const char*, int ,int);
char* my_generator(const char*,int);
char * dupstr (char*);
void *xmalloc (int);
void parse(char *s);
int test(char *s , struct hubo *h);
char* getArg(string s, int argNum);
void hubo_update(hubo_ref_t *h_ref, hubo_state_t *h_state);
int name2mot(char*s, hubo_param_t *h);
double hubo_get(char*s, hubo_ref_t *h, hubo_param_t *p);
void hubo_jmc_beep(hubo_param_t *h, hubo_board_cmd_t *c, char* buff);
void hubo_jmc_home(hubo_param_t *h, hubo_board_cmd_t *c, char* buff);
//char* cmd [] ={ "test","hello", "world", "hell" ,"word", "quit", " " };
void hubo_jmc_home_all(hubo_param_t *h, hubo_board_cmd_t *c, char* buff);
void hubo_enc_reset(hubo_param_t *h, hubo_board_cmd_t *c, int jnt);
void hubo_startup_all(hubo_param_t *h, hubo_board_cmd_t *c, char* buff);
int name2sensor(char* name, hubo_param_t *h);
double hubo_set(char*s, hubo_param_t *p);
char* cmd [] ={ "initialize","fet","initializeAll","homeAll","zero","zeroacc","iniSensors","reset",
                "ctrl","ctrlAll","enczero", "goto","get","test","update", "quit","beep", "home"," ",
                "resetAll","status"}; //,


int main() {
	printf("\n");
	printf(" ***************** hubo-ach **************** \n");
	printf(" Support: Daniel M. Lofaro dan@danlofaro.com \n");
	printf(" ******************************************* \n");

	// get initial values for hubo
	// open ach channel
	int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME, NULL);
	assert( ACH_OK == r );

	// open hubo state
	r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME, NULL);
	assert( ACH_OK == r );

       // initialize control channel
       r = ach_open(&chan_hubo_board_cmd, HUBO_CHAN_BOARD_CMD_NAME, NULL);
       assert( ACH_OK == r );

        // get initial values for hubo
        hubo_ref_t H_ref;
        hubo_board_cmd_t H_cmd;
        hubo_state_t H_state;
        hubo_param_t H_param;
        memset( &H_ref,   0, sizeof(H_ref));
        memset( &H_cmd,  0, sizeof(H_cmd));
        memset( &H_state, 0, sizeof(H_state));
        memset( &H_param, 0, sizeof(H_param));
	
	usleep(250000);

	// set default values for Hubo
	setJointParams(&H_param, &H_state);

	size_t fs;
	r = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
	assert( sizeof(H_ref) == fs );
	r = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
	assert( sizeof(H_state) == fs );

    char *buf;
    rl_attempted_completion_function = my_completion;
    printf("\n");
    while((buf = readline(">> hubo-ach: "))!=NULL) {
        //enable auto-complete
        rl_bind_key('\t',rl_complete);

        printf("   ");

        /* get update after every command */
        hubo_update(&H_ref, &H_state);
        
        int tsleep = 0;
        char* buf0 = getArg(buf, 0);

        if (strcmp(buf0,"update")==0) {
            hubo_update(&H_ref, &H_state);
            printf("--->Hubo Information Updated\n");
        }
        else if (strcmp(buf0,"get")==0) {
            double jRef = hubo_get(buf,&H_ref, &H_param);
            char* tmp = getArg(buf,1);
            printf(">> %s = %f rad \n",tmp,jRef);
        }
        else if (strcmp(buf0,"goto")==0) {
            int jnt = hubo_set(buf, &H_param);
            float f = 0.0;
            char* str = getArg(buf,2);
            if(sscanf(str, "%f", &f) != 0){  //It's a float.
                H_ref.ref[jnt] = (double)f;
                int r = ach_put( &chan_hubo_ref, &H_ref, sizeof(H_ref) );
                char* tmp = getArg(buf,1);
                printf(">> %s = %f rad \n",tmp,f);
            }
            else {
                printf(">> Bad input \n");
            }
        }
        else if (strcmp(buf0,"beep")==0) {
            hubo_jmc_beep(&H_param, &H_cmd, buf);
        }
        else if (strcmp(buf0,"home")==0) {
            hubo_jmc_home(&H_param, &H_cmd, buf);
            printf("%s - Home \n",getArg(buf,1));
        }
        else if (strcmp(buf0,"homeAll")==0) {
            hubo_jmc_home_all(&H_param, &H_cmd, buf);
            printf("%s - Home All \n",getArg(buf,1));
            tsleep = 5;
            
        }

	else if (strcmp(buf0,"ctrl")==0) {
		int hOnOff = atof(getArg(buf,2));
		if(hOnOff == 0 | hOnOff == 1) {
			H_cmd.type = D_CTRL_ON_OFF;
			H_cmd.joint = name2mot(getArg(buf,1),&H_param);  // set motor num
			if( hOnOff == 1)
			    H_cmd.param[0] = D_ENABLE;         // 1 = on
			if( hOnOff == 0)
			    H_cmd.param[0] = D_DISABLE;         // 0 = off
                        if( hOnOff == 1 | hOnOff == 0 )
                            r = ach_put( &chan_hubo_board_cmd, &H_cmd, sizeof(H_cmd) );
			if( hOnOff == 0) {
				printf("%s - Turning Off CTRL\n",getArg(buf,1));}
			else {
				printf("%s - Turning On CTRL\n",getArg(buf,1));}
		}

	}

	else if (strcmp(buf0,"ctrlAll")==0) {
		int hOnOff = atof(getArg(buf,1));
		if( hOnOff == 0 | hOnOff == 1) {
			H_cmd.type = D_CTRL_ON_OFF_ALL;
			if( hOnOff == 1)
			    H_cmd.param[0] = D_ENABLE;         // 1 = on
			if( hOnOff == 0)
			    H_cmd.param[0] = D_DISABLE;         // 0 = off
                        if( hOnOff == 1 | hOnOff == 0 )
                            r = ach_put( &chan_hubo_board_cmd, &H_cmd, sizeof(H_cmd) );
			if( hOnOff == 0) {
				printf("Turning Off ALL CTRL\n");}
			else {
				printf("Turning On ALL CTRL\n");}
		}

	}

        else if (strcmp(buf0,"status")==0) {
            H_cmd.type = D_GET_STATUS;
            H_cmd.joint = name2mot(getArg(buf,1),&H_param);  // set motor num
            r = ach_put( &chan_hubo_board_cmd, &H_cmd, sizeof(H_cmd) );
            printf("%s - Getting Status \n", &H_param.joint[H_cmd.joint].name);
            usleep(50*1000);
            hubo_update(&H_ref, &H_state);
            hubo_joint_status_t e = H_state.status[H_cmd.joint];
            printf("Mode         : %d \n", H_ref.mode[H_cmd.joint]);
            printf("Zeroed       : %d \n", H_state.joint[H_cmd.joint].zeroed);
            printf("Homed        : %d \n", e.homeFlag);
            printf("Jam          : %d \n", e.jam);
            printf("PWM Saturated: %d \n", e.pwmSaturated);
            printf("Big Error    : %d \n", e.bigError);
            printf("Enc Error    : %d \n", e.encError);
            printf("Drive Fault  : %d \n", e.driverFault);
            printf("Pos Err (min): %d \n", e.posMinError);
            printf("Pos Err (max): %d \n", e.posMaxError);
            printf("Velos Error  : %d \n", e.velError);
            printf("Acc Error    : %d \n", e.accError);
            printf("Temp Error   : %d \n", e.tempError);
            printf("Active       : %d \n", H_state.joint[H_cmd.joint].active);
      	}
        else if (strcmp(buf0,"resetAll")==0) {
            for( int i = 0; i < HUBO_JOINT_COUNT; i++) {
                hubo_enc_reset(&H_param, &H_cmd, i);
                printf("%s - Resetting Encoder \n", &H_param.joint[i].name);
                usleep(10*1000);
            }
	}

        else if (strcmp(buf0,"reset")==0) {
            int jnt = name2mot(getArg(buf, 1), &H_param); 
            hubo_enc_reset(&H_param, &H_cmd, jnt);
            printf("%s - Resetting Encoder \n",getArg(buf,1));
        }
        else if (strcmp(buf0,"startup")==0) {
            hubo_startup_all(&H_param, &H_cmd, buf);
            printf("Starting up Hubo\n");
            tsleep = 2;
        }
        else if (strcmp(buf0,"ctrl")==0) {
            int onOrOff = atof(getArg(buf,2));
            if(onOrOff == 0 | onOrOff == 1) {
                H_cmd.type = D_CTRL_SWITCH;
                H_cmd.joint = name2mot(getArg(buf,1),&H_param);  // set motor num
                if(onOrOff==1)			// 1 = on, 0 = 0ff
                    H_cmd.param[0] = D_ENABLE;
                else if(onOrOff==0)
                    H_cmd.param[0] = D_DISABLE;	
                r = ach_put( &chan_hubo_board_cmd, &H_cmd, sizeof(H_cmd) );
                if(onOrOff == 0) {
                    printf("%s - Turning Off CTRL\n",getArg(buf,1));}
                else {
                    printf("%s - Turning On CTRL\n",getArg(buf,1));}
            }
        }
        else if (strcmp(buf0,"fet")==0) {
            int onOrOff = atof(getArg(buf,2));
            if(onOrOff == 0 | onOrOff == 1) {
                H_cmd.type = D_FET_SWITCH;
                H_cmd.joint = name2mot(getArg(buf,1),&H_param);  // set motor num
                if(onOrOff==1)
                    H_cmd.param[0] = D_ENABLE;
                else if(onOrOff==0)
                    H_cmd.param[0] = D_DISABLE;
                int r = ach_put( &chan_hubo_board_cmd, &H_cmd, sizeof(H_cmd) );
                if(onOrOff == 0) {
                    printf("%s - Turning Off FET\n",getArg(buf,1));}
                else {
                    printf("%s - Turning On FET\n",getArg(buf,1));}
            }
        }
        else if (strcmp(buf0,"initialize")==0) {
            H_cmd.type = D_JMC_INITIALIZE;
            H_cmd.joint = name2mot(getArg(buf,1),&H_param);	// set motor num
            //C.val[0] = atof(getArg(buf,2));
            int r = ach_put( &chan_hubo_board_cmd, &H_cmd, sizeof(H_cmd) );
            printf("%s - Initialize \n",getArg(buf,1));
        }
        else if (strcmp(buf0,"initializeAll")==0) {
            H_cmd.type = D_JMC_INITIALIZE_ALL;
            int r = ach_put( &chan_hubo_board_cmd, &H_cmd, sizeof(H_cmd) );
            printf("%s - Initialize All\n",getArg(buf,1));
            tsleep = 2;
        }
        else if (strcmp(buf0,"zero")==0) {
            int ft = name2sensor(getArg(buf,1), &H_param);
            H_cmd.type = D_NULL_SENSOR;
            switch(ft){
                case HUBO_FT_R_HAND: H_cmd.param[0] = D_R_HAND_FT; break;
                case HUBO_FT_L_HAND: H_cmd.param[0] = D_L_HAND_FT; break;
                case HUBO_FT_R_FOOT: H_cmd.param[0] = D_R_FOOT_FT; break;
                case HUBO_FT_L_FOOT: H_cmd.param[0] = D_L_FOOT_FT; break;
                case HUBO_IMU0: H_cmd.param[0] = D_IMU_SENSOR_0; break;
                case HUBO_IMU1: H_cmd.param[0] = D_IMU_SENSOR_1; break;
                case HUBO_IMU2: H_cmd.param[0] = D_IMU_SENSOR_2; break;
                    printf("Name %s not found!\n", getArg(buf,1));
            }
            ach_put( &chan_hubo_board_cmd, &H_cmd, sizeof(H_cmd) );
        }
        else if (strcmp(buf0,"zeroacc")==0) {
            int ft = name2sensor(getArg(buf,1), &H_param);
            H_cmd.type = D_NULL_SENSOR;
            switch(ft){
                case HUBO_FT_R_FOOT: H_cmd.param[0] = D_R_FOOT_ACC; break;
                case HUBO_FT_L_FOOT: H_cmd.param[0] = D_L_FOOT_ACC; break;
                case HUBO_IMU0: H_cmd.param[0] = D_IMU_SENSOR_0; break;
                case HUBO_IMU1: H_cmd.param[0] = D_IMU_SENSOR_1; break;
                case HUBO_IMU2: H_cmd.param[0] = D_IMU_SENSOR_2; break;
                default:
                    printf("Name %s not found!\n", getArg(buf,1));
            }
            ach_put( &chan_hubo_board_cmd, &H_cmd, sizeof(H_cmd) );
        }
        else if (strcmp(buf0,"iniSensors")==0){
            printf("Nulling All Sensors\n");
            H_cmd.type = D_NULL_SENSORS_ALL;
            int r = ach_put( &chan_hubo_board_cmd, &H_cmd, sizeof(H_cmd));
            
        }
        /* Quit */
        else if (strcmp(buf0,"quit")==0)
            break;
        if (buf[0]!=0)
        add_history(buf);
        sleep(tsleep);	// sleep for tsleep sec
    }
    free(buf);
    return 0;
}

double hubo_get(char*s, hubo_ref_t *h, hubo_param_t *p) {

	/* get joint number */
	int jointNo = name2mot(getArg(s,1),p);

	return h->ref[jointNo];
}

double hubo_set(char*s, hubo_param_t *p) {

    /* get joint number */
    int jointNo = name2mot(getArg(s,1),p);
    return jointNo;
}

void hubo_jmc_beep(hubo_param_t *h, hubo_board_cmd_t *c, char* buff) {
        /* make beep */
        c->type = D_JMC_BEEP;
        c->joint = name2mot(getArg(buff, 1), h);
        c->dValues[0] = atof(getArg(buff,2));
        int r = ach_put( &chan_hubo_board_cmd, c, sizeof(*c) );
        printf("send beep r = %i C = %i v = %f\n",r, (int)c->type, c->dValues[0]);

}

void hubo_jmc_home(hubo_param_t *h, hubo_board_cmd_t *c, char* buff) {
        /* make beiep */
        c->type = D_GOTO_HOME;
        c->joint = name2mot(getArg(buff, 1), h);
        int r = ach_put( &chan_hubo_board_cmd, c, sizeof(*c) );
//	printf(">> Home %s \n",getArg(buff,1));
}

void hubo_enc_reset(hubo_param_t *h, hubo_board_cmd_t *c, int jnt) {
        c->type = D_ZERO_ENCODER;
//        c->joint = name2mot(getArg(buff, 1), h);
        c->joint = jnt;
        int r= ach_put( &chan_hubo_board_cmd, c, sizeof(*c) );
}
void hubo_jmc_home_all(hubo_param_t *h, hubo_board_cmd_t *c, char* buff) {
    /* make beiep */
    c->type = D_GOTO_HOME_ALL;
    int r = ach_put( &chan_hubo_board_cmd, c, sizeof(*c) );
//	printf(">> Home %s \n",getArg(buff,1));
}
void hubo_update(hubo_ref_t *h_ref, hubo_state_t *h_state) {
    size_t fs;
    int r = ach_get( &chan_hubo_ref, h_ref, sizeof(*h_ref), &fs, NULL, ACH_O_LAST );
    if((r == ACH_OK) | (r == ACH_MISSED_FRAME)) {
            assert( sizeof(*h_ref) == fs );}
    r = ach_get( &chan_hubo_state, h_state, sizeof(*h_state), &fs, NULL, ACH_O_LAST );
    if((r == ACH_OK) | (r == ACH_MISSED_FRAME)) {
            assert( sizeof(*h_state) == fs );}
    // look into posix message que
    // posix rt signal can give signal number and an interger
}

void hubo_startup_all(hubo_param_t *h, hubo_board_cmd_t *c, char* buff) {

	c->type = D_SENSOR_STARTUP;
	int r = ach_put( &chan_hubo_board_cmd, c, sizeof(*c));

}

char* getArg(string s, int argNum) {
    istringstream iss(s);

    int i = 0;
    do
    {
        string sub;
        iss >> sub;
        if( i == argNum ) {
                return (char*)sub.c_str(); }
        i++;
    } while (iss);

    return NULL;
}

int test(char* s, hubo_param_t *h) {
    printf("\n dan test\n");

    char* mot = getArg(s,1);

    int tmp = name2mot(mot, h);
    printf("mot = %i \n",tmp);
    return 0;
}

int name2mot(char* name, hubo_param_t *h) {
    /* Returns the number of the requested joint */
    int i = 0;
    int iout = -1;
    for( i = 0; i < HUBO_JOINT_COUNT ; i++ ) {
        char *mot = h->joint[i].name;
        if (strcmp(name, mot) == 0) {
        	iout = i;
		}
    }
	return iout;
}


int name2sensor(char* name, hubo_param_t *h) {
	/* Returns the number of the requested joint */
	int i = 0;
	int iout = -1;
	for( i = 0; i < HUBO_SENSOR_COUNT ; i++ ) {
	    char *sens = h->sensor[i].name;
	    printf("i = %i, name = %s\n", i,sens);
	    if (strcmp(name, sens) == 0) {
		iout = i;}
	}
	return iout;
}


static char** my_completion( const char * text , int start,  int end) {
    char **matches;

    matches = (char **)NULL;

    if (start == 0)
            matches = rl_completion_matches ((char*)text, &my_generator);
    else
            rl_bind_key('\t',rl_abort);

    return (matches);
}

char* my_generator(const char* text, int state) {
    static int list_index, len;
    char *name;

    if (!state) {
            list_index = 0;
            len = strlen (text);
    }

    while (name = cmd[list_index]) {
            list_index++;

    if (strncmp (name, text, len) == 0)
            return (dupstr(name));
    }

    /* If no names matched, then return NULL. */
    return ((char *)NULL);
}

char * dupstr (char* s) {
    char *r;

    r = (char*) xmalloc ((strlen (s) + 1));
    strcpy (r, s);
    return (r);
}

void * xmalloc (int size) {
    void *buf;

    buf = malloc (size);
    if (!buf) {
            fprintf (stderr, "Error: Out of memory. Exiting.'n");
            exit (1);
    }
    return buf;
}
